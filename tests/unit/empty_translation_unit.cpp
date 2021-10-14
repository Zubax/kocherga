// This file is needed to work around an apparent limitation of SonarCloud: it seems to be unable to analyze headers
// that are not part of any translation unit. This empty C++ file is therefore added to host header fields with the
// help of the "-include" compiler option (that forces inclusion of header files into translation units).
// Find details in the build script and at https://community.sonarsource.com/t/analyzing-a-header-only-c-library/51468

#include "generated_header_that_includes_all_kocherga_headers.hpp"
