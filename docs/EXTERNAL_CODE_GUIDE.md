# Integrating External Research Code on Windows

Integrating "research quality" code (like InfiniTAM, ORB-SLAM, or random GitHub repos) into a Windows build is notoriously difficult. Research code is often Linux-centric, relies on GCC-specific flags, and ignores Windows DLL export rules.

This guide provides the battle-tested workflow for Windows C++/CMake environments.

---

## 1. The Strategy: "Fork, Submodule, and Patch"

**Do not** just download zip files. **Do not** copy-paste code directly into `src/` unless it's a single header file. You need a way to pull upstream updates while keeping your Windows fixes.

### Workflow

1. **Fork** the repo to your own GitHub account (e.g., `yourname/InfiniTAM`)
2. **Create** a `windows-port` branch in your fork
3. **Add as submodule** to your playground:
   ```bash
   git submodule add https://github.com/yourname/InfiniTAM external/InfiniTAM
   ```
4. **Modify** the code inside `external/InfiniTAM` to fix build errors
5. **Commit** changes to your fork

This allows you to merge changes from the original author later while keeping your Windows patches applied.

---

## 2. Build Integration: `add_subdirectory` (Not ExternalProject)

In your main `CMakeLists.txt`, try to include the project directly. This puts the external project into your IDE solution, allowing you to debug inside their code easily.

```cmake
# Root CMakeLists.txt

# 1. Define variables the external project expects BEFORE including it
# Many research projects assume they are the 'root' project.
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# 2. Add the subdirectory
add_subdirectory(external/InfiniTAM)

# 3. Link your playground to their library
target_link_libraries(my_slam_engine PRIVATE InfiniTAM)
```

### If the External CMake is Garbage (Common in Research)

**Do not fight their CMake.** Write a "Shim" CMake file.

1. Create `external/InfiniTAM_wrapper/CMakeLists.txt`
2. Manually define a library target that includes their source files
3. Include that wrapper instead of their raw repo

---

## 3. The "Windows Magic" CMake Flags

Research code usually fails on Windows for three specific reasons:

### A. The "Missing Symbols" Error (DLL Hell)

On Linux, all symbols are visible. On Windows, you must explicitly mark functions as `__declspec(dllexport)`. Retrofitting this to a large codebase is a nightmare.

**The Fix:** Force CMake to export everything automatically.

```cmake
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
```

*Note: This works for 95% of cases. It fails for static global variables, but it's the best starting point.*

### B. The "Compiler Flag" Error

They will have `-Wall -O3 -march=native` in their CMake. These are GCC/Clang flags. MSVC will error out.

**The Fix:** Use `if(MSVC)` to sanitize flags.

```cmake
# Inside your wrapper or their modified CMakeLists.txt
if(MSVC)
    # Remove GCC flags if they hardcoded them
    string(REPLACE "-Wall" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-O3" "/O2" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

    # Add necessary defines for math constants (M_PI, etc.)
    add_compile_definitions(_USE_MATH_DEFINES)

    # Disable "unsafe" function warnings (sprintf, etc.)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()
```

### C. The "Linux Header" Error

If they include `<unistd.h>`, `<sys/time.h>`, or `<pthread.h>`, the build will fail.

**The Fix:**

- **Pthreads**: Add `pthreads` to your `vcpkg.json`. Vcpkg provides a Windows-compatible pthread wrapper.
- **Unistd**: Create a dummy header `compat/unistd.h` containing `#include <io.h>` and `#include <process.h>` and add `compat/` to your include path.

---

## 4. Handling CUDA (InfiniTAM/NeRF)

If the external project uses CUDA, Windows introduces a specific pain point: **architecture version matching**.

If you install PyTorch (which comes with CUDA binaries) and then try to compile custom C++ CUDA code, they often conflict.

**Advice:** Ensure the CUDA Toolkit installed on your Windows machine matches the major version of the CUDA used by any pre-compiled binaries (like PyTorch) you plan to load in Python.

In CMake, enable CUDA as a language:

```cmake
enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES "native")  # Optimizes for your specific GPU
```

---

## 5. Dependency Management (The "Diamond Problem")

If InfiniTAM needs OpenCV, and you need OpenCV, you must ensure you are using the **exact same DLLs**.

### Delete Their "3rdparty" Folder

Many research projects bundle a specific version of Eigen or OpenCV inside a folder. **Delete it.**

### Force Vcpkg

1. Ensure both your project and their CMake script find packages via `find_package(OpenCV REQUIRED)`
2. When you configure their project via `add_subdirectory`, because you are using the Vcpkg toolchain file in your root, the sub-project will automatically use the Vcpkg versions of libraries
3. This solves the version conflict

---

## Summary Checklist for Windows Modification

- [ ] Fork the repo
- [ ] Submodule it into your project (`external/`)
- [ ] Set `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS` to `ON`
- [ ] Add `_CRT_SECURE_NO_WARNINGS` and `_USE_MATH_DEFINES`
- [ ] Use Vcpkg for dependencies (OpenCV, Eigen, g2o) so both your code and their code link against the same DLL
- [ ] Delete any bundled `3rdparty/` folders in external code
- [ ] Create compatibility headers for Linux-specific includes if needed
- [ ] Match CUDA Toolkit version with PyTorch if using both

---

## Example: Adding InfiniTAM

```bash
# 1. Fork on GitHub, then:
git submodule add https://github.com/yourname/InfiniTAM external/InfiniTAM

# 2. Create wrapper
mkdir external/InfiniTAM_wrapper
```

`external/InfiniTAM_wrapper/CMakeLists.txt`:
```cmake
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
endif()

# Include their code
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../InfiniTAM ${CMAKE_CURRENT_BINARY_DIR}/InfiniTAM)
```

Root `CMakeLists.txt`:
```cmake
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(external/InfiniTAM_wrapper)
target_link_libraries(my_target PRIVATE InfiniTAM::Engine)
```
