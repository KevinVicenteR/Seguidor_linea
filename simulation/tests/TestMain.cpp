#include "TestFramework.h"

int main() {
  int failed = 0;
  for (const test::TestCase& testCase : test::registry()) {
    try {
      testCase.body();
      std::cout << "  ok    " << testCase.name << "\n";
    } catch (const test::Failure& failure) {
      failed++;
      std::cout << "  FALLA " << testCase.name << "\n        " << failure.message << "\n";
    } catch (const std::exception& error) {
      failed++;
      std::cout << "  FALLA " << testCase.name << "\n        excepción: " << error.what() << "\n";
    }
  }
  std::cout << "\n" << test::registry().size() - failed << "/" << test::registry().size() << " pruebas aprobadas\n";
  return failed == 0 ? 0 : 1;
}
