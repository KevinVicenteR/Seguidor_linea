/*
  TestFramework.h - Mini framework de pruebas sin dependencias externas.
*/
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace test {

struct TestCase {
  std::string name;
  std::function<void()> body;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

struct Registrar {
  Registrar(const std::string& name, std::function<void()> body) { registry().push_back({name, std::move(body)}); }
};

struct Failure {
  std::string message;
};

inline void fail(const std::string& expression, const char* file, int line) {
  std::ostringstream out;
  out << file << ":" << line << ": " << expression;
  throw Failure{out.str()};
}

}  // namespace test

#define TEST_CONCAT_INNER(a, b) a##b
#define TEST_CONCAT(a, b) TEST_CONCAT_INNER(a, b)
#define TEST(name)                                                                   \
  static void TEST_CONCAT(test_fn_, __LINE__)();                                     \
  static test::Registrar TEST_CONCAT(test_reg_, __LINE__)(name, TEST_CONCAT(test_fn_, __LINE__)); \
  static void TEST_CONCAT(test_fn_, __LINE__)()

#define EXPECT_TRUE(cond) \
  do { if (!(cond)) test::fail("se esperaba verdadero: " #cond, __FILE__, __LINE__); } while (0)
#define EXPECT_FALSE(cond) \
  do { if (cond) test::fail("se esperaba falso: " #cond, __FILE__, __LINE__); } while (0)
#define EXPECT_EQ(actual, expected)                                                               \
  do {                                                                                            \
    const auto a_ = (actual);                                                                     \
    const auto e_ = (expected);                                                                   \
    if (!(a_ == e_)) {                                                                            \
      std::ostringstream o_;                                                                      \
      o_ << #actual " == " #expected " (valor " << +a_ << ", esperado " << +e_ << ")";            \
      test::fail(o_.str(), __FILE__, __LINE__);                                                   \
    }                                                                                             \
  } while (0)
#define EXPECT_NEAR(actual, expected, tolerance)                                                  \
  do {                                                                                            \
    const double a_ = (actual);                                                                   \
    const double e_ = (expected);                                                                 \
    if (std::fabs(a_ - e_) > (tolerance)) {                                                       \
      std::ostringstream o_;                                                                      \
      o_ << #actual " ~= " #expected " (valor " << a_ << ", esperado " << e_ << ")";              \
      test::fail(o_.str(), __FILE__, __LINE__);                                                   \
    }                                                                                             \
  } while (0)
#define EXPECT_THROWS(statement)                                                                  \
  do {                                                                                            \
    bool threw_ = false;                                                                          \
    try { statement; } catch (...) { threw_ = true; }                                             \
    if (!threw_) test::fail("se esperaba una excepción: " #statement, __FILE__, __LINE__);        \
  } while (0)
