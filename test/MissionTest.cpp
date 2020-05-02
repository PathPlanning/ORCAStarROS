#include "gtest/gtest.h"
#include "Mission.h"



class TestMission : public ::testing::Test
{
    protected:
        void SetUp()
        {

            task = new Mission("test.xml", 2, 5000);


        }

        void TearDown()
        {
            delete task;

        }
        Mission *task;

};

TEST_F(TestMission, SimpleTest)
{
    bool res1 = task->ReadTask();
    ASSERT_TRUE(true);

    auto result = task->StartMission();
    float sr = 100;
    float time = 3;
    float allTime = 6;
    int c = 0, co = 0;

    ASSERT_FLOAT_EQ(result.successRate, sr);
    ASSERT_FLOAT_EQ(result.flowTime, allTime);
    ASSERT_FLOAT_EQ(result.makeSpan, time);
    ASSERT_EQ(result.collisions, c);
    ASSERT_EQ(result.collisionsObst, co);
}
