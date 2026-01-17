// src/index/rtree_sqlite.cpp
//
// Step 3 implementation: revert per-scan rebuild to DROP+CREATE (Reset) instead of DELETE-all.
//
// Rationale:
// - For SQLite RTree, `DELETE FROM rtree_tracks;` can be as expensive as (or worse than)
//   dropping and recreating the virtual table.
// - This version keeps your prepared statements approach, but rebuilds by calling Reset().

#include "rtree_sqlite.h"

#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace idx {

struct SqliteRTreeIndex::Impl {
  sqlite3* db = nullptr;
  sqlite3_stmt* insert_stmt = nullptr;
  sqlite3_stmt* update_stmt = nullptr;
  sqlite3_stmt* query_stmt = nullptr;
  sqlite3_stmt* clear_stmt = nullptr;  // kept if you still have ClearAll() in the header
  sqlite3_stmt* state_stmt = nullptr;
  sqlite3_stmt* cov_stmt = nullptr;
  sqlite3_stmt* meta_stmt = nullptr;
  std::size_t count = 0;
};

static void exec_or_throw(sqlite3* db, const std::string& sql) {
  char* err = nullptr;
  const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &err);
  if (rc != SQLITE_OK) {
    std::string msg = err ? err : "sqlite3_exec failed";
    if (err) sqlite3_free(err);
    throw std::runtime_error(msg + " (sql=" + sql + ")");
  }
}

static inline void insert_point_stmt_or_throw(sqlite3_stmt* stmt,
                                              std::uint64_t id,
                                              double x,
                                              double y,
                                              double z,
                                              bool clear_bindings) {
  if (!stmt) throw std::runtime_error("insert_point_stmt_or_throw: stmt is null");

  sqlite3_reset(stmt);

  if (clear_bindings) {
    sqlite3_clear_bindings(stmt);
  }

  sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(id));
  sqlite3_bind_double(stmt, 2, x);
  sqlite3_bind_double(stmt, 3, x);
  sqlite3_bind_double(stmt, 4, y);
  sqlite3_bind_double(stmt, 5, y);
  sqlite3_bind_double(stmt, 6, z);
  sqlite3_bind_double(stmt, 7, z);

  const int rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step(insert) failed");
  }
}

SqliteRTreeIndex::SqliteRTreeIndex() : p_(new Impl()) {
  const int rc = sqlite3_open(":memory:", &p_->db);
  if (rc != SQLITE_OK) {
    throw std::runtime_error("sqlite3_open(:memory:) failed");
  }

  // Pragmas for in-memory speed
  exec_or_throw(p_->db, "PRAGMA temp_store=MEMORY;");
  exec_or_throw(p_->db, "PRAGMA synchronous=OFF;");
  exec_or_throw(p_->db, "PRAGMA journal_mode=MEMORY;");

  Reset();
}

SqliteRTreeIndex::SqliteRTreeIndex(const std::string& db_uri) : p_(new Impl()) {
  const char* uri = db_uri.empty() ? ":memory:" : db_uri.c_str();
  if (!db_uri.empty() && db_uri != ":memory:") {
    std::error_code ec;
    std::filesystem::path p(db_uri);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path(), ec);
    }
  }

  const int rc = sqlite3_open(uri, &p_->db);
  if (rc != SQLITE_OK) {
    const char* msg = p_->db ? sqlite3_errmsg(p_->db) : "unknown";
    throw std::runtime_error(std::string("sqlite3_open failed: ") + msg);
  }

  if (db_uri.empty() || db_uri == ":memory:") {
    exec_or_throw(p_->db, "PRAGMA temp_store=MEMORY;");
    exec_or_throw(p_->db, "PRAGMA synchronous=OFF;");
    exec_or_throw(p_->db, "PRAGMA journal_mode=MEMORY;");
  } else {
    exec_or_throw(p_->db, "PRAGMA journal_mode=WAL;");
    exec_or_throw(p_->db, "PRAGMA synchronous=NORMAL;");
  }

  Reset();
}

SqliteRTreeIndex::~SqliteRTreeIndex() {
  if (!p_) return;
  if (p_->insert_stmt) sqlite3_finalize(p_->insert_stmt);
  if (p_->update_stmt) sqlite3_finalize(p_->update_stmt);
  if (p_->query_stmt) sqlite3_finalize(p_->query_stmt);
  if (p_->clear_stmt) sqlite3_finalize(p_->clear_stmt);
  if (p_->state_stmt) sqlite3_finalize(p_->state_stmt);
  if (p_->cov_stmt) sqlite3_finalize(p_->cov_stmt);
  if (p_->meta_stmt) sqlite3_finalize(p_->meta_stmt);
  if (p_->db) sqlite3_close(p_->db);
  delete p_;
  p_ = nullptr;
}

void SqliteRTreeIndex::Reset() {
  if (!p_ || !p_->db) throw std::runtime_error("SqliteRTreeIndex not initialized");

  if (p_->insert_stmt) {
    sqlite3_finalize(p_->insert_stmt);
    p_->insert_stmt = nullptr;
  }
  if (p_->query_stmt) {
    sqlite3_finalize(p_->query_stmt);
    p_->query_stmt = nullptr;
  }
  if (p_->update_stmt) {
    sqlite3_finalize(p_->update_stmt);
    p_->update_stmt = nullptr;
  }
  if (p_->clear_stmt) {
    sqlite3_finalize(p_->clear_stmt);
    p_->clear_stmt = nullptr;
  }
  if (p_->state_stmt) {
    sqlite3_finalize(p_->state_stmt);
    p_->state_stmt = nullptr;
  }
  if (p_->cov_stmt) {
    sqlite3_finalize(p_->cov_stmt);
    p_->cov_stmt = nullptr;
  }
  if (p_->meta_stmt) {
    sqlite3_finalize(p_->meta_stmt);
    p_->meta_stmt = nullptr;
  }
  p_->count = 0;

  exec_or_throw(p_->db, "DROP TABLE IF EXISTS rtree_tracks;");
  exec_or_throw(p_->db,
                "CREATE VIRTUAL TABLE rtree_tracks USING rtree("
                "id, minX, maxX, minY, maxY, minZ, maxZ"
                ");");

  // Insert prepared statement
  const char* insert_sql = "INSERT INTO rtree_tracks VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7);";
  if (sqlite3_prepare_v2(p_->db, insert_sql, -1, &p_->insert_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 insert failed");
  }

  // Update-or-insert prepared statement (safe for first insert)
  const char* update_sql =
      "INSERT OR REPLACE INTO rtree_tracks VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7);";
  if (sqlite3_prepare_v2(p_->db, update_sql, -1, &p_->update_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 update failed");
  }

  // Overlap query prepared statement
  const char* query_sql =
      "SELECT id FROM rtree_tracks WHERE "
      "minX <= ?2 AND maxX >= ?1 AND "
      "minY <= ?4 AND maxY >= ?3 AND "
      "minZ <= ?6 AND maxZ >= ?5;";
  if (sqlite3_prepare_v2(p_->db, query_sql, -1, &p_->query_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 query failed");
  }

  // Optional: keep ClearAll() functional if your header declares it.
  // Not used by BuildFrom* in "Step 3" mode.
  const char* clear_sql = "DELETE FROM rtree_tracks;";
  if (sqlite3_prepare_v2(p_->db, clear_sql, -1, &p_->clear_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 clear failed");
  }
}

// If you kept ClearAll() in the header, keep this implementation.
// Not used for Step 3 rebuild, but harmless and can be used later for experiments.
void SqliteRTreeIndex::ClearAll() {
  if (!p_ || !p_->db || !p_->clear_stmt) throw std::runtime_error("ClearAll called before Reset");

  sqlite3_reset(p_->clear_stmt);
  const int rc = sqlite3_step(p_->clear_stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step(clear) failed");
  }
  sqlite3_reset(p_->clear_stmt);
  p_->count = 0;
}

void SqliteRTreeIndex::Begin() {
  if (!p_ || !p_->db) throw std::runtime_error("Begin called before Reset");
  exec_or_throw(p_->db, "BEGIN IMMEDIATE;");
}

void SqliteRTreeIndex::Commit() {
  if (!p_ || !p_->db) throw std::runtime_error("Commit called before Reset");
  exec_or_throw(p_->db, "COMMIT;");
}

void SqliteRTreeIndex::Rollback() {
  if (!p_ || !p_->db) throw std::runtime_error("Rollback called before Reset");
  exec_or_throw(p_->db, "ROLLBACK;");
}

void SqliteRTreeIndex::InsertPoint(std::uint64_t id, double x, double y, double z) {
  if (!p_ || !p_->insert_stmt) throw std::runtime_error("InsertPoint called before Reset");
  insert_point_stmt_or_throw(p_->insert_stmt, id, x, y, z, /*clear_bindings=*/true);
  p_->count++;
}

void SqliteRTreeIndex::UpdatePoint(std::uint64_t id, double x, double y, double z) {
  if (!p_ || !p_->update_stmt) throw std::runtime_error("UpdatePoint called before Reset");
  insert_point_stmt_or_throw(p_->update_stmt, id, x, y, z, /*clear_bindings=*/true);
  if (p_->count < static_cast<std::size_t>(id + 1)) {
    p_->count = static_cast<std::size_t>(id + 1);
  }
}

void SqliteRTreeIndex::BuildFromXYZ(const double* xs,
                                    const double* ys,
                                    const double* zs,
                                    std::size_t n) {
  if (!xs || !ys || !zs) throw std::runtime_error("BuildFromXYZ: null input pointers");

  // STEP 3: drop + create + re-prepare statements per build
  Reset();

  exec_or_throw(p_->db, "BEGIN IMMEDIATE;");
  for (std::size_t i = 0; i < n; ++i) {
    insert_point_stmt_or_throw(p_->insert_stmt,
                               static_cast<std::uint64_t>(i),
                               xs[i],
                               ys[i],
                               zs[i],
                               /*clear_bindings=*/false);
    p_->count++;
  }
  exec_or_throw(p_->db, "COMMIT;");
}

void SqliteRTreeIndex::BuildFromInterleavedXYZ(const double* state,
                                               std::size_t n_tracks,
                                               std::size_t stride,
                                               std::size_t x_off,
                                               std::size_t y_off,
                                               std::size_t z_off) {
  if (!state) {
    throw std::runtime_error("BuildFromInterleavedXYZ: state is null");
  }
  if (stride == 0) {
    throw std::runtime_error("BuildFromInterleavedXYZ: stride is 0");
  }
  if (x_off >= stride || y_off >= stride || z_off >= stride) {
    std::ostringstream oss;
    oss << "BuildFromInterleavedXYZ: offsets out of range. stride=" << stride
        << " x_off=" << x_off << " y_off=" << y_off << " z_off=" << z_off;
    throw std::runtime_error(oss.str());
  }

  // STEP 3: drop + create + re-prepare statements per build
  Reset();

  exec_or_throw(p_->db, "BEGIN IMMEDIATE;");
  for (std::size_t i = 0; i < n_tracks; ++i) {
    const std::size_t base = i * stride;
    const double x = state[base + x_off];
    const double y = state[base + y_off];
    const double z = state[base + z_off];

    insert_point_stmt_or_throw(p_->insert_stmt,
                               static_cast<std::uint64_t>(i),
                               x,
                               y,
                               z,
                               /*clear_bindings=*/false);
    p_->count++;
  }
  exec_or_throw(p_->db, "COMMIT;");
}

std::vector<std::uint64_t> SqliteRTreeIndex::QueryAabb(const EcefAabb& aabb) const {
  if (!p_ || !p_->query_stmt) throw std::runtime_error("QueryAabb called before Reset");
  sqlite3_reset(p_->query_stmt);

  sqlite3_bind_double(p_->query_stmt, 1, aabb.min_x);
  sqlite3_bind_double(p_->query_stmt, 2, aabb.max_x);
  sqlite3_bind_double(p_->query_stmt, 3, aabb.min_y);
  sqlite3_bind_double(p_->query_stmt, 4, aabb.max_y);
  sqlite3_bind_double(p_->query_stmt, 5, aabb.min_z);
  sqlite3_bind_double(p_->query_stmt, 6, aabb.max_z);

  std::vector<std::uint64_t> ids;
  while (true) {
    const int rc = sqlite3_step(p_->query_stmt);
    if (rc == SQLITE_ROW) {
      const sqlite3_int64 id = sqlite3_column_int64(p_->query_stmt, 0);
      ids.push_back(static_cast<std::uint64_t>(id));
    } else if (rc == SQLITE_DONE) {
      break;
    } else {
      throw std::runtime_error("sqlite3_step(query) failed");
    }
  }
  return ids;
}

std::size_t SqliteRTreeIndex::size() const { return p_ ? p_->count : 0; }

void SqliteRTreeIndex::EnsureMetaTable() {
  if (!p_ || !p_->db) throw std::runtime_error("EnsureMetaTable called before Reset");
  exec_or_throw(p_->db,
                "CREATE TABLE IF NOT EXISTS tracker_meta("
                "key TEXT PRIMARY KEY, value TEXT"
                ");");
}

std::string SqliteRTreeIndex::GetMeta(const std::string& key) const {
  if (!p_ || !p_->db) throw std::runtime_error("GetMeta called before Reset");
  const char* sql = "SELECT value FROM tracker_meta WHERE key = ?1;";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(p_->db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 meta select failed");
  }
  sqlite3_bind_text(stmt, 1, key.c_str(), -1, SQLITE_TRANSIENT);
  std::string value;
  const int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    const unsigned char* txt = sqlite3_column_text(stmt, 0);
    if (txt) value = reinterpret_cast<const char*>(txt);
  } else if (rc != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error("sqlite3_step(meta select) failed");
  }
  sqlite3_finalize(stmt);
  return value;
}

void SqliteRTreeIndex::SetMeta(const std::string& key, const std::string& value) {
  if (!p_ || !p_->db) throw std::runtime_error("SetMeta called before Reset");
  const char* sql = "INSERT OR REPLACE INTO tracker_meta(key,value) VALUES(?1,?2);";
  sqlite3_stmt* stmt = nullptr;
  if (sqlite3_prepare_v2(p_->db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 meta upsert failed");
  }
  sqlite3_bind_text(stmt, 1, key.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, value.c_str(), -1, SQLITE_TRANSIENT);
  const int rc = sqlite3_step(stmt);
  if (rc != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error("sqlite3_step(meta upsert) failed");
  }
  sqlite3_finalize(stmt);
}

static std::string build_cov_upsert_sql() {
  std::ostringstream cols;
  std::ostringstream vals;
  cols << "id";
  vals << "?1";
  int param = 2;
  for (int r = 0; r < 9; ++r) {
    for (int c = r; c < 9; ++c) {
      cols << ",p" << r << c;
      vals << ",?" << param++;
    }
  }
  std::ostringstream sql;
  sql << "INSERT OR REPLACE INTO track_cov(" << cols.str() << ") VALUES(" << vals.str() << ");";
  return sql.str();
}

void SqliteRTreeIndex::EnsureTrackTables() {
  if (!p_ || !p_->db) throw std::runtime_error("EnsureTrackTables called before Reset");
  exec_or_throw(p_->db,
                "CREATE TABLE IF NOT EXISTS track_state("
                "id INTEGER PRIMARY KEY,"
                "x REAL,y REAL,z REAL,vx REAL,vy REAL,vz REAL,ax REAL,ay REAL,az REAL,"
                "t_last_update REAL,t_pred REAL"
                ");");
  exec_or_throw(p_->db,
                "CREATE TABLE IF NOT EXISTS track_meta("
                "id INTEGER PRIMARY KEY,"
                "status INTEGER,"
                "quality REAL"
                ");");
  exec_or_throw(p_->db,
                "CREATE TABLE IF NOT EXISTS track_cov("
                "id INTEGER PRIMARY KEY,"
                "p00 REAL,p01 REAL,p02 REAL,p03 REAL,p04 REAL,p05 REAL,p06 REAL,p07 REAL,p08 REAL,"
                "p11 REAL,p12 REAL,p13 REAL,p14 REAL,p15 REAL,p16 REAL,p17 REAL,p18 REAL,"
                "p22 REAL,p23 REAL,p24 REAL,p25 REAL,p26 REAL,p27 REAL,p28 REAL,"
                "p33 REAL,p34 REAL,p35 REAL,p36 REAL,p37 REAL,p38 REAL,"
                "p44 REAL,p45 REAL,p46 REAL,p47 REAL,p48 REAL,"
                "p55 REAL,p56 REAL,p57 REAL,p58 REAL,"
                "p66 REAL,p67 REAL,p68 REAL,"
                "p77 REAL,p78 REAL,"
                "p88 REAL"
                ");");

  if (!p_->state_stmt) {
    const char* sql =
      "INSERT OR REPLACE INTO track_state("
      "id,x,y,z,vx,vy,vz,ax,ay,az,t_last_update,t_pred"
      ") VALUES (?1,?2,?3,?4,?5,?6,?7,?8,?9,?10,?11,?12);";
    if (sqlite3_prepare_v2(p_->db, sql, -1, &p_->state_stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("sqlite3_prepare_v2 track_state upsert failed");
    }
  }
  if (!p_->meta_stmt) {
    const char* sql =
      "INSERT OR REPLACE INTO track_meta(id,status,quality) VALUES (?1,?2,?3);";
    if (sqlite3_prepare_v2(p_->db, sql, -1, &p_->meta_stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("sqlite3_prepare_v2 track_meta upsert failed");
    }
  }
  if (!p_->cov_stmt) {
    const std::string sql = build_cov_upsert_sql();
    if (sqlite3_prepare_v2(p_->db, sql.c_str(), -1, &p_->cov_stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("sqlite3_prepare_v2 track_cov upsert failed");
    }
  }
}

void SqliteRTreeIndex::UpsertTrackState(std::uint64_t id, const double* x9, double t_last_update, double t_pred) {
  if (!p_ || !p_->state_stmt) throw std::runtime_error("UpsertTrackState called before EnsureTrackTables");
  if (!x9) throw std::runtime_error("UpsertTrackState: null state");
  sqlite3_reset(p_->state_stmt);
  sqlite3_clear_bindings(p_->state_stmt);
  sqlite3_bind_int64(p_->state_stmt, 1, static_cast<sqlite3_int64>(id));
  for (int k = 0; k < 9; ++k) {
    sqlite3_bind_double(p_->state_stmt, 2 + k, x9[k]);
  }
  sqlite3_bind_double(p_->state_stmt, 11, t_last_update);
  sqlite3_bind_double(p_->state_stmt, 12, t_pred);
  const int rc = sqlite3_step(p_->state_stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step(track_state upsert) failed");
  }
}

void SqliteRTreeIndex::UpsertTrackCovUpper(std::uint64_t id, const double* cov_upper45) {
  if (!p_ || !p_->cov_stmt) throw std::runtime_error("UpsertTrackCovUpper called before EnsureTrackTables");
  if (!cov_upper45) throw std::runtime_error("UpsertTrackCovUpper: null covariance");
  sqlite3_reset(p_->cov_stmt);
  sqlite3_clear_bindings(p_->cov_stmt);
  sqlite3_bind_int64(p_->cov_stmt, 1, static_cast<sqlite3_int64>(id));
  for (int k = 0; k < 45; ++k) {
    sqlite3_bind_double(p_->cov_stmt, 2 + k, cov_upper45[k]);
  }
  const int rc = sqlite3_step(p_->cov_stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step(track_cov upsert) failed");
  }
}

void SqliteRTreeIndex::UpsertTrackMeta(std::uint64_t id, int status, double quality) {
  if (!p_ || !p_->meta_stmt) throw std::runtime_error("UpsertTrackMeta called before EnsureTrackTables");
  sqlite3_reset(p_->meta_stmt);
  sqlite3_clear_bindings(p_->meta_stmt);
  sqlite3_bind_int64(p_->meta_stmt, 1, static_cast<sqlite3_int64>(id));
  sqlite3_bind_int(p_->meta_stmt, 2, status);
  sqlite3_bind_double(p_->meta_stmt, 3, quality);
  const int rc = sqlite3_step(p_->meta_stmt);
  if (rc != SQLITE_DONE) {
    throw std::runtime_error("sqlite3_step(track_meta upsert) failed");
  }
}

}  // namespace idx
