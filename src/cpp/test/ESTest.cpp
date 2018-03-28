// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "test/ESTest.hpp"

#include "ES/Service.hpp"

class ESTest : public testing::Test {
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(ESTest, TestTest) { EXPECT_TRUE(true); }
