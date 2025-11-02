# Offline app build using the MKL dev base
FROM devbase:mkl
WORKDIR /src

# Expect project files:
#   CMakeLists.txt
#   include/la.h
#   src/la_eigen.cpp
#   src/la_mkl.cpp
#   src/main.cpp
COPY . /src

# Build (MKL backend only, C++98)
#RUN cmake -S . -B /build -G Ninja \
#    -DUSE_MKL=ON -DCMAKE_CXX_STANDARD=98 \
#    -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake && \
#    cmake --build /build -j

RUN cmake -S . -B /build -G Ninja \
    -DUSE_MKL=ON -DCMAKE_CXX_STANDARD=98 -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/intel/oneapi/mkl/latest/lib/cmake && \
    cmake --build /build -j

# --- Docs generation (online) ---
# Set to 0 to skip docs at build time.
ARG GENERATE_DOCS=1

RUN if [ "$GENERATE_DOCS" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends doxygen graphviz && \
      rm -rf /var/lib/apt/lists/* && \
      doxygen -g /src/Doxyfile && \
      sed -i 's|^OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = /src/docs|' /src/Doxyfile && \
      sed -i 's|^RECURSIVE.*|RECURSIVE = YES|' /src/Doxyfile && \
      sed -i 's|^INPUT .*|INPUT = include src|' /src/Doxyfile && \
      sed -i 's|^GENERATE_LATEX.*|GENERATE_LATEX = NO|' /src/Doxyfile && \
      sed -i 's|^HAVE_DOT.*|HAVE_DOT = YES|' /src/Doxyfile && \
      sed -i 's|^DOT_IMAGE_FORMAT.*|DOT_IMAGE_FORMAT = svg|' /src/Doxyfile && \
      doxygen /src/Doxyfile ; \
    fi

ENTRYPOINT ["/build/demo"]
