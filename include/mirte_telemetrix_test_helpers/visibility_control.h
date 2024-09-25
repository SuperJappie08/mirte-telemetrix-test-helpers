// Based on:
//
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MIRTE_TELEMETRIX_TEST_HELPERS__VISIBILITY_CONTROL_H_
#define MIRTE_TELEMETRIX_TEST_HELPERS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MIRTE_TELEMETRIX_TEST_HELPERS_EXPORT __attribute__((dllexport))
#define MIRTE_TELEMETRIX_TEST_HELPERS_IMPORT __attribute__((dllimport))
#else
#define MIRTE_TELEMETRIX_TEST_HELPERS_EXPORT __declspec(dllexport)
#define MIRTE_TELEMETRIX_TEST_HELPERS_IMPORT __declspec(dllimport)
#endif
#ifdef MIRTE_TELEMETRIX_TEST_HELPERS_BUILDING_DLL
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC MIRTE_TELEMETRIX_TEST_HELPERS_EXPORT
#else
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC MIRTE_TELEMETRIX_TEST_HELPERS_IMPORT
#endif
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC_TYPE MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC
#define MIRTE_TELEMETRIX_TEST_HELPERS_LOCAL
#else
#define MIRTE_TELEMETRIX_TEST_HELPERS_EXPORT __attribute__((visibility("default")))
#define MIRTE_TELEMETRIX_TEST_HELPERS_IMPORT
#if __GNUC__ >= 4
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC __attribute__((visibility("default")))
#define MIRTE_TELEMETRIX_TEST_HELPERS_LOCAL __attribute__((visibility("hidden")))
#else
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC
#define MIRTE_TELEMETRIX_TEST_HELPERS_LOCAL
#endif
#define MIRTE_TELEMETRIX_TEST_HELPERS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MIRTE_TELEMETRIX_TEST_HELPERS__VISIBILITY_CONTROL_H_