#include "tests/gtest/gtest.h"
#include "utilities.h"
#include "core/parallel.h"
#include <atomic>


TEST(Parallel, Basics) {
    Parallel::init();

    std::atomic<int> counter{0};
    Parallel::forLoop([&](int64_t) { ++counter; }, 1000, 1);
    EXPECT_EQ(1000, counter);

    counter = 0;
    Parallel::forLoop([&](int64_t) { ++counter; }, 1000, 19);
    EXPECT_EQ(1000, counter);

    counter = 0;
    Parallel::forLoop2D([&](Point2i p) { ++counter; }, Point2i(15, 14));
    EXPECT_EQ(15*14, counter);

    Parallel::cleanup();
}

TEST(Parallel, DoNothing) {
    Parallel::init();

    std::atomic<int> counter{0};
    Parallel::forLoop([&](int64_t) { ++counter; }, 0);
    EXPECT_EQ(0, counter);

    counter = 0;
    Parallel::forLoop2D([&](Point2i p) { ++counter; }, Point2i(0, 0));
    EXPECT_EQ(0, counter);

    Parallel::cleanup();
}
