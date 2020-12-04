#pragma once

// Copyright 2009 Google Inc. All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: vladl@google.com (Vlad Losev)

// This sample shows how to use Google Test listener API to implement
// an alternative console output and how to use the UnitTest reflection API
// to enumerate test cases and tests and to inspect their results.

#include <stdio.h>

#include "colored_printf.hpp"
#include "gtest/gtest.h"

using ::testing::EmptyTestEventListener;
using ::testing::InitGoogleTest;
using ::testing::Test;
using ::testing::TestCase;
using ::testing::TestEventListeners;
using ::testing::TestInfo;
using ::testing::TestPartResult;
using ::testing::UnitTest;

namespace gtest_tools
{
// Provides alternative output mode which produces minimal amount of
// information about tests.
class Printer : public EmptyTestEventListener
{
public:
  explicit Printer(bool redirectToStdErr = false)
    : redirectToStdErr_(redirectToStdErr), printerStream_(redirectToStdErr ? stderr : stdout), oldStdOutStream_(nullptr)
  {
  }

private:
  // Called before any test activity starts.
  void OnTestProgramStart(const UnitTest& unit_test) override
  {
    colored_fprintf(printerStream_, Color::BPurple, "\n\n\nRunning %d tests from %d test cases.\n",
                    unit_test.GetInstance()->test_to_run_count(), unit_test.GetInstance()->test_case_to_run_count());
    colored_fprintf(printerStream_, Color::BYellow, "Warning: You have %d disabled tests.\n\n",
                    unit_test.GetInstance()->disabled_test_count());

    for (int i = 0; i < unit_test.GetInstance()->total_test_case_count(); i++)
    {
      colored_fprintf(printerStream_, Color::BWhite, "%s:\n", unit_test.GetInstance()->GetTestCase(i)->name());
      for (int j = 0; j < unit_test.GetInstance()->GetTestCase(i)->total_test_count(); j++)
      {
        fprintf(printerStream_, "\t - %s \n", unit_test.GetInstance()->GetTestCase(i)->GetTestInfo(j)->name());
      }
      fprintf(printerStream_, "\n");
    }
    fflush(printerStream_);
  }

  // Called after all test activities have ended.
  void OnTestProgramEnd(const UnitTest& unit_test) override
  {
    unit_test.Passed() ? colored_fprintf(printerStream_, Color::BGreen, "\n\n TEST PROGRAM SUCCEEDED! \n\n\n") :
                         colored_fprintf(printerStream_, Color::BRed, "\n\n TEST PROGRAM FAILED! \n\n\n");
    fflush(printerStream_);
  }

  // Called before a test starts.
  void OnTestStart(const TestInfo& test_info) override
  {
    colored_fprintf_brackets(printerStream_, Color::Green, "RUN", Color::Cyan, "%s::%s.\n", test_info.test_case_name(),
                             test_info.name());
    fflush(printerStream_);

    if (redirectToStdErr_)
    {
      oldStdOutStream_ = stdout;
      fflush(oldStdOutStream_);
      stdout = stderr;
    }
  }

  // Called after a failed assertion or a SUCCEED() invocation.
  void OnTestPartResult(const TestPartResult& test_part_result) override
  {
    if (test_part_result.failed())
      colored_fprintf_brackets(printerStream_, Color::Red, "X", Color::Yellow, "\n%s\n", test_part_result.message());
    fflush(printerStream_);
  }

  // Called after a test ends.
  void OnTestEnd(const TestInfo& test_info) override
  {
    test_info.result()->Failed() ? colored_brackets(printerStream_, Color::Red, "FAILED") :
                                   colored_brackets(printerStream_, Color::Green, "PASSED");
    colored_fprintf(printerStream_, Color::Blue, "%s::%s (%d ms).\n\n", test_info.test_case_name(), test_info.name(),
                    static_cast<int>(test_info.result()->elapsed_time()));
    fflush(printerStream_);

    if (redirectToStdErr_)
    {
      fflush(stderr);
      stdout = oldStdOutStream_;
      fflush(stdout);
    }
  }

private:
  bool redirectToStdErr_;
  FILE* printerStream_;
  FILE* oldStdOutStream_;
};  // class Printer

}  // namespace gtest_tools
