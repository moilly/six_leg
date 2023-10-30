#include <QtTest>

#include <iostream>
#include "hexapod_kinematics.h"

// add necessary includes here

class AlgoTest : public QObject
{
    Q_OBJECT

public:
    AlgoTest();
    ~AlgoTest();

private slots:
    void initTestCase();
    void cleanupTestCase();
    void test_case();

private:
    double max_delta_length;
    double delta_lengths;

    Eigen::Vector3d ini_pos;
    Eigen::Vector3d ini_rpy;
    Eigen::Vector3d pos;
    Eigen::Vector3d rpy;

    hexkins::HexapodConfig config;
};

AlgoTest::AlgoTest()
{

}

AlgoTest::~AlgoTest()
{

}

void AlgoTest::initTestCase()
{
    config.base_joints = {{
        { 5.468, -1.777, 0 },
        { -1.981, -5.498, 0 },
        { -3.921, -4.205, 0 },
        { -3.921, 4.205, 0 },
        { -1.981, 5.498, 0 },
        { 5.468, 1.777, 0 },
    }};

    config.platform_joints = {{
        { 3.175, -3, 0 },
        { 1.01, -4.25, 0 },
        { -4.185, -1.25, 0 },
        { -4.185, 1.25, 0 },
        { 1.01, 4.25, 0 },
        { 3.175, 3, 0 },
    }};

    config.kins_fwd_max_retries = 50;

    max_delta_length = 0.05;
    delta_lengths = 0.3;

}

void AlgoTest::cleanupTestCase()
{

}

void AlgoTest::test_case()
{
    int test_num = 100;
    // calculate by inverse_kinematics && use in forward_kinematics
    std::array<double, 6> strut_lengths;

    auto generator = QRandomGenerator::global();

    // iterator 100 times
    for (int i = 0; i < test_num; i++) {
        pos.x() = generator->bounded(20.0);
        pos.y() = generator->bounded(20.0);
        pos.z() = generator->bounded(20.0);

        rpy.x() = generator->bounded(0.1);
        rpy.y() = -generator->bounded(0.1);
        rpy.z() = generator->bounded(0.1);

        strut_lengths = hexkins::inverse_kinematics(config, pos, hexkins::rpy_to_quaternion(rpy));

        Eigen::Vector3d temp_pos;
        Eigen::Vector3d temp_rpy;
        Eigen::Quaterniond temp_ori;
        std::tie(temp_pos, temp_ori) = hexkins::forward_kinematics(config, strut_lengths, pos, hexkins::rpy_to_quaternion(rpy));
        temp_rpy = hexkins::Quaterniond2EulerAngles(temp_ori);

        QCOMPARE(QString::number(pos.x(), 'f', 10), QString::number(temp_pos.x(), 'f', 10));
        QCOMPARE(QString::number(pos.y(), 'f', 10), QString::number(temp_pos.y(), 'f', 10));
        QCOMPARE(QString::number(pos.z(), 'f', 10), QString::number(temp_pos.z(), 'f', 10));

        QCOMPARE(QString::number(rpy.x(), 'f', 10), QString::number(temp_rpy.x(), 'f', 10));
        QCOMPARE(QString::number(rpy.y(), 'f', 10), QString::number(temp_rpy.y(), 'f', 10));
        QCOMPARE(QString::number(rpy.z(), 'f', 10), QString::number(temp_rpy.z(), 'f', 10));
    }
}

QTEST_APPLESS_MAIN(AlgoTest)

#include "tst_algotest.moc"
