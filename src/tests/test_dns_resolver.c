/*
 * Copyright (c) 2023, Erich Styger
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "platform.h"
#if PL_CONFIG_USE_UNIT_TESTS
#include "test_dns_resolver.h"
#include "McuUtility.h"
#include "dns_resolver.h"
#include "unity/unity.h"
#include "fff.h"

DEFINE_FFF_GLOBALS;
FAKE_VOID_FUNC(DnsResolver_Init);

void TestDnsResolver_Test(void) {
  DnsResolver_info_t info;
  int res;
  unsigned char buf[16];

  res = DnsResolver_ResolveName("127.0.0.1", &info, 0);
  TEST_ASSERT_MESSAGE(res==0, "fixed IP does not need resolving");
  ip4addr_ntoa_r(&info.resolved_addr, buf, sizeof(buf));
  TEST_ASSERT(McuUtility_strcmp(buf, "127.0.0.1")==0);
}

#endif /* PL_CONFIG_USE_UNIT_TESTS */