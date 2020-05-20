#include "gtest/gtest.h"
#include "XMLReader.h"
#include "XMLLogger.h"


using namespace tinyxml2;

class TestXmlIO : public ::testing::Test
{
    protected:
        void SetUp()
        {


            grid ={{0,0,0,0},
                   {0,1,0,0},
                   {0,0,1,0},
                   {0,0,0,0}};

            obstacles =
                    {
                       {{0,0},{4,0},{4, 4},{0, 4}}
                    };

            underTest1 = new XMLReader("test.xml");

            sum = Summary(100,10,100,200,2,1);

            stepsLog.insert({0, {Point(1,1)}});
            stepsLog.insert({1, {Point(2,2)}});

            goalsLog.insert({0, {Point(1,1)}});
            goalsLog.insert({1, {Point(2,2)}});

            resultLog.insert({0, {true, 1}});
            resultLog.insert({1, {true, 1}});

            underTest2 = new XMLLogger("test_output.xml", "test.xml");
            underTest2->SetSummary(sum);
            underTest2->SetResults(stepsLog, goalsLog, resultLog);
            underTest2->GenerateLog();


            readres =  underTest1->ReadData() &&
                            underTest1->GetMap(&map) &&
                            underTest1->GetAgents(agents, 2) &&
                            underTest1->GetEnvironmentOptions(&options);
        }
        void TearDown()
        {
            delete underTest1;
            delete underTest2;
            delete map;
            delete options;
            for(size_t i = 0; i < agents.size(); i++)
            {
                delete agents[i];
            }
        }
        XMLReader *underTest1;
        XMLLogger *underTest2;
        std::vector<std::vector<int>> grid;
        std::vector<std::vector<Point>> obstacles;
        bool readres;
        Summary sum;
        Map *map;
        std::vector<Agent*> agents;
        EnvironmentOptions *options;

        std::unordered_map<int, std::vector<Point>> stepsLog;
        std::unordered_map<int, std::vector<Point>> goalsLog;
        std::unordered_map<int, std::pair<bool, int>> resultLog;

};


TEST_F(TestXmlIO, inputTest)
{
    ASSERT_TRUE(readres);
}


TEST_F(TestXmlIO, agentsReadTest)
{
    ASSERT_EQ(agents.size(), 2);
    AgentParam pref = AgentParam(3.0, 5.4, 33.0, 0.3, CN_DEFAULT_REPS , 1.0, 10);

    auto param = agents[0]->GetParam();
    ASSERT_FLOAT_EQ(param.sightRadius, pref.sightRadius);
    ASSERT_FLOAT_EQ(param.timeBoundary, pref.timeBoundary);
    ASSERT_FLOAT_EQ(param.timeBoundaryObst, pref.timeBoundaryObst);
    ASSERT_FLOAT_EQ(param.radius, pref.radius);
    ASSERT_FLOAT_EQ(param.rEps, pref.rEps);
    ASSERT_FLOAT_EQ(param.maxSpeed, pref.maxSpeed);
    EXPECT_EQ(param.agentsMaxNum, pref.agentsMaxNum);


    param = agents[1]->GetParam();
    ASSERT_FLOAT_EQ(param.sightRadius, pref.sightRadius);
    ASSERT_FLOAT_EQ(param.timeBoundary, pref.timeBoundary);
    ASSERT_FLOAT_EQ(param.timeBoundaryObst, pref.timeBoundaryObst);
    ASSERT_FLOAT_EQ(param.radius, pref.radius);
    ASSERT_FLOAT_EQ(param.rEps, pref.rEps);
    ASSERT_FLOAT_EQ(param.maxSpeed, pref.maxSpeed);
    EXPECT_EQ(param.agentsMaxNum, pref.agentsMaxNum);

    auto id = agents[0]->GetID();
    auto start = agents[0]->GetStart();
    auto goal = agents[0]->GetGoal();

    ASSERT_TRUE(id == 0);
    ASSERT_TRUE(start == Point(0.5, 0.5));
    ASSERT_TRUE(goal == Point(3.5, 0.5));


    id = agents[1]->GetID();
    start = agents[1]->GetStart();
    goal = agents[1]->GetGoal();

    ASSERT_TRUE(id == 1);
    ASSERT_TRUE(start == Point(3.5, 3.5));
    ASSERT_TRUE(goal == Point(0.5, 3.5));



}


TEST_F(TestXmlIO, mapReadTest)
{
    auto obst = map->GetObstacles();
    EXPECT_EQ(obst.size(), 1);
    EXPECT_EQ(obst[0].size(), 4);

    for(int i = 0; i < 4; i++)
    {
        ASSERT_TRUE(obst[0][i].right == obstacles[0][i] );
    }



    for(size_t i = 0; i < grid.size(); i++)
    {
        for(size_t j = 0; j < grid[i].size(); j++)
        {
            EXPECT_EQ(map->CellIsObstacle(i, j), grid[i][j]);
            EXPECT_NE(map->CellIsTraversable(i, j), grid[i][j]);
        }
    }


}


TEST_F(TestXmlIO, optionsReadTest)
{
    EXPECT_EQ(options->breakingties, 0);
    EXPECT_EQ(options->allowsqueeze, false);
    EXPECT_EQ(options->cutcorners, false);
    ASSERT_FLOAT_EQ(options->hweight, 1);
    ASSERT_FLOAT_EQ(options->timestep, 0.1);
    ASSERT_FLOAT_EQ(options->delta, 0.1);
}



TEST_F(TestXmlIO, outputTest)
{

    XMLDocument *doc = new XMLDocument();
    ASSERT_TRUE(doc->LoadFile("test_output.xml") == XMLError::XML_SUCCESS);

    XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);

    XMLElement *log = root->FirstChildElement(CNS_TAG_LOG);


    XMLElement *summary = log->FirstChildElement(CNS_TAG_SUM);
    float sr, time, ft, ms;
    int c, co;

    ASSERT_TRUE(summary->QueryFloatAttribute(CNS_TAG_ATTR_SR, &sr) == XMLError::XML_SUCCESS);
    ASSERT_FLOAT_EQ(sr, sum.successRate);

    ASSERT_TRUE(summary->QueryFloatAttribute(CNS_TAG_ATTR_RUNTIME, &time) == XMLError::XML_SUCCESS);
    ASSERT_FLOAT_EQ(time, sum.runTime);

    ASSERT_TRUE(summary->QueryFloatAttribute(CNS_TAG_ATTR_FLOWTIME, &ft) == XMLError::XML_SUCCESS);
    ASSERT_FLOAT_EQ(ft, sum.flowTime);

    ASSERT_TRUE(summary->QueryFloatAttribute(CNS_TAG_ATTR_MAKESPAN, &ms) == XMLError::XML_SUCCESS);
    ASSERT_FLOAT_EQ(ms, sum.makeSpan);


    ASSERT_TRUE(summary->QueryIntAttribute(CNS_TAG_ATTR_COL_AGNT, &c) == XMLError::XML_SUCCESS);
    ASSERT_EQ(c, sum.collisions);

    ASSERT_TRUE(summary->QueryIntAttribute(CNS_TAG_ATTR_COL_OBST, &co) == XMLError::XML_SUCCESS);
    ASSERT_EQ(co, sum.collisionsObst);


    for (auto agnode = log->FirstChildElement(CNS_TAG_AGENT); agnode; agnode = agnode->NextSiblingElement(CNS_TAG_AGENT))
    {
        int id;

        ASSERT_TRUE(agnode->QueryIntAttribute(CNS_TAG_ATTR_ID, &id) == XMLError::XML_SUCCESS);

        auto path = agnode->FirstChildElement(CNS_TAG_PATH);

        bool r;
        int st;



        ASSERT_TRUE(path->QueryBoolAttribute(CNS_TAG_ATTR_PATHFOUND, &r) == XMLError::XML_SUCCESS);
        ASSERT_TRUE(path->QueryIntAttribute(CNS_TAG_ATTR_STEPS, &st) == XMLError::XML_SUCCESS);
        ASSERT_EQ(r, resultLog[id].first);
        ASSERT_EQ(st, resultLog[id].second);

        auto step = path->FirstChildElement(CNS_TAG_STEP);

        int n;
        float x, y;

        ASSERT_TRUE(step->QueryIntAttribute(CNS_TAG_ATTR_NUM, &n) == XMLError::XML_SUCCESS);
        ASSERT_EQ(n, 0);

        ASSERT_TRUE(step->QueryFloatAttribute(CNS_TAG_ATTR_X, &x) == XMLError::XML_SUCCESS);
        ASSERT_TRUE(step->QueryFloatAttribute(CNS_TAG_ATTR_Y, &y) == XMLError::XML_SUCCESS);

        ASSERT_TRUE(Point(x, y) == stepsLog[id][0]);


        ASSERT_TRUE(step->QueryFloatAttribute("next.xr", &x) == XMLError::XML_SUCCESS);
        ASSERT_TRUE(step->QueryFloatAttribute("next.yr", &y) == XMLError::XML_SUCCESS);

        ASSERT_TRUE(Point(x, y) == goalsLog[id][0]);
    }
}
