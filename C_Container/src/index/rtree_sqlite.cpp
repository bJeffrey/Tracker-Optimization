// src/index/rtree_sqlite.cpp
#include "index/rtree_sqlite.h"

#include <sqlite3.h>

#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace idx {

struct SqliteRTreeIndex::Impl {
  sqlite3* db = nullptr;
  sqlite3_stmt* insert_stmt = nullptr;
  sqlite3_stmt* query_stmt = nullptr;
  sqlite3_stmt* clear_stmt = nullptr;  // NEW
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

  // Reset the prepared statement for reuse.
  sqlite3_reset(stmt);

  // In tight loops, clearing bindings is unnecessary if we bind all params every time.
  // Keep it optional for safety in case InsertPoint() changes later.
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

  // Optional but useful for in-memory speed; safe for :memory: usage.
  exec_or_throw(p_->db, "PRAGMA temp_store=MEMORY;");
  exec_or_throw(p_->db, "PRAGMA synchronous=OFF;");
  exec_or_throw(p_->db, "PRAGMA journal_mode=MEMORY;");

  Reset();
}

SqliteRTreeIndex::~SqliteRTreeIndex() {
  if (!p_) return;
  if (p_->insert_stmt) sqlite3_finalize(p_->insert_stmt);
  if (p_->query_stmt) sqlite3_finalize(p_->query_stmt);
  if (p_->clear_stmt) sqlite3_finalize(p_->clear_stmt);  // NEW
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
  if (p_->clear_stmt) {  // NEW
    sqlite3_finalize(p_->clear_stmt);
    p_->clear_stmt = nullptr;
  }
  p_->count = 0;

  exec_or_throw(p_->db, "DROP TABLE IF EXISTS rtree_tracks;");
  exec_or_throw(p_->db,
                "CREATE VIRTUAL TABLE rtree_tracks USING rtree("
                "id, minX, maxX, minY, maxY, minZ, maxZ"
                ");");

  // Prepared statements:
  // Insert: id, minX,maxX,minY,maxY,minZ,maxZ
  const char* insert_sql = "INSERT INTO rtree_tracks VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7);";
  if (sqlite3_prepare_v2(p_->db, insert_sql, -1, &p_->insert_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 insert failed");
  }

  // Overlap query:
  // Find boxes that overlap [min,max] in each dimension.
  const char* query_sql =
      "SELECT id FROM rtree_tracks WHERE "
      "minX <= ?2 AND maxX >= ?1 AND "
      "minY <= ?4 AND maxY >= ?3 AND "
      "minZ <= ?6 AND maxZ >= ?5;";
  if (sqlite3_prepare_v2(p_->db, query_sql, -1, &p_->query_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 query failed");
  }

  // NEW: clear all rows without dropping schema / re-preparing other statements
  const char* clear_sql = "DELETE FROM rtree_tracks;";
  if (sqlite3_prepare_v2(p_->db, clear_sql, -1, &p_->clear_stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite3_prepare_v2 clear failed");
  }
}

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

void SqliteRTreeIndex::InsertPoint(std::uint64_t id, double x, double y, double z) {
  if (!p_ || !p_->insert_stmt) throw std::runtime_error("InsertPoint called before Reset");
  insert_point_stmt_or_throw(p_->insert_stmt, id, x, y, z, /*clear_bindings=*/true);
  p_->count++;
}

void SqliteRTreeIndex::BuildFromXYZ(const double* xs,
                                    const double* ys,
                                    const double* zs,
                                    std::size_t n) {
  if (!xs || !ys || !zs) throw std::runtime_error("BuildFromXYZ: null input pointers");
  if (!p_ || !p_->db || !p_->insert_stmt) throw std::runtime_error("BuildFromXYZ called before Reset");

  

  exec_or_throw(p_->db, "BEGIN IMMEDIATE;");  // NEW: IMMEDIATE for write-heavy cycles
  // NEW: don't drop/recreate schema; just clear existing rows
  ClearAll();
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

  exec_or_throw(p_->db, "BEGIN IMMEDIATE;");  // NEW
  
  // NEW: don't drop/recreate schema; just clear existing rows
  ClearAll();
  
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

  // We always bind all query parameters, so no need to sqlite3_clear_bindings() here.
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

}  // namespace idx
