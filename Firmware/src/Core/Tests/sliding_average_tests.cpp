#include <cmath>

#include "sliding_average.h"
#include <gtest/gtest.h>
#include <vector>

class SlidingAvgTests : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

TEST_F(SlidingAvgTests, small_check)
{
    int ab = 0;
    (void) ab;
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}