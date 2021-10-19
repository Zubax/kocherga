#define CATCH_CONFIG_MAIN
#include "catch.hpp"  // NOLINT
#include "util.hpp"

auto kocherga::getRandomByte() -> std::uint8_t
{
    return util::getRandomInteger<std::uint8_t>();
}