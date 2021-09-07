# Kocherga contribution guide

## Directory layout

The production sources are located under `/kocherga/`.
Do not put anything else in there.

The tests are located under `/tests/`.
This directory also contains the top `CMakeLists.txt` needed to build and run the tests on the local machine.

The high-level documentation is written in the main README, API documentation is given directly in the header files.

## Standards

The library shall be implemented in ISO C++17 with partial adherence to MISRA C++.
The MISRA compliance is enforced by Clang-Tidy and SonarQube.
Deviations are documented directly in the source code as follows:

```c
// Intentional violation of MISRA: <some valid reason>
<... deviant construct ...>  // NOLINT NOSONAR
```

The full list of deviations with the accompanying explanation can be found by grepping the sources.

Do not suppress compliance warnings using the means provided by static analysis tools because such deviations
are impossible to track at the source code level.
An exception applies for the case of false-positive (invalid) warnings -- those should not be mentioned in the codebase.

[Zubax C++ Coding Conventions](https://kb.zubax.com/x/84Ah) shall be followed.
Formatting is enforced by Clang-Format; it is used also to fail the CI/CD build if violations are detected.

Unfortunately, some rules are hard or impractical to enforce automatically,
so code reviewers shall be aware of MISRA and general high-reliability coding practices
to prevent non-compliant code from being accepted into upstream.

## Tools

For the full list of the tools please refer to the CI scripts.

### Clang-Tidy

Clang-Tidy is used to enforce compliance with MISRA and Zubax Coding Conventions.

Clang-Tidy is invoked automatically on each translation unit before it is compiled;
the build will fail if the tool is not available locally.
To disable this behavior, pass `NO_STATIC_ANALYSIS=1` to CMake at the generation time.

### Clang-Format

Clang-Format is used to enforce compliance with MISRA and Zubax Coding Conventions.

To reformat the sources, generate the project and build the target `format`; e.g., for Make: `make format`.

### SonarQube

SonarQube is a cloud solution so its use is delegated to the CI/CD pipeline.
If you need access, please get in touch with the maintainers.

### IDE

The recommended development environment is JetBrains CLion. The root project file can be found under `tests/`.
The repository contains the spelling dictionaries for CLion located under `.idea/`, be sure to use them.

## Testing

Generate the CMake project, build all, and then build the target `test` (e.g., `make test`).

Some of the tests are intended to be run manually due to lack of adequate automation solutions in the v0 ecosystem.
Please navigate to `/tests/integration/validator/` for details.

## Releasing

1. Bump the version numbers (`KOCHERGA_VERSION_MAJOR`, `KOCHERGA_VERSION_MINOR`) in `kocherga.hpp`. Push the change.
2. Create a new release on GitHub: <https://github.com/Zubax/kocherga/releases/new>
