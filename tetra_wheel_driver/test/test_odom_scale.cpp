#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <unistd.h>

#include "../src/odom_scale.hpp"

using odom_scale::Pose2D;
using odom_scale::Scale;
using odom_scale::ScaledOdometry;

namespace
{
// 보드가 30 Hz 로 내는 것처럼 원호를 잘게 나눠 넣는다
Pose2D DriveArc(ScaledOdometry & odom, Pose2D board, const Scale & scale, double distance, double turn, int steps)
{
	Pose2D out;
	for(int i = 0; i < steps; i++)
	{
		const double mid = board.theta + turn / steps / 2.0;
		board.x += std::cos(mid) * distance / steps;
		board.y += std::sin(mid) * distance / steps;
		board.theta = odom_scale::NormalizeAngle(board.theta + turn / steps);
		out = odom.Update(board, scale);
	}
	return out;
}
}  // namespace

TEST(OdomScale, IdentityPassesBoardThrough)
{
	ScaledOdometry odom;
	const Pose2D board{1.234, -0.5, 2.9};
	odom.Update(Pose2D{}, Scale{});
	const Pose2D out = odom.Update(board, Scale{});
	EXPECT_DOUBLE_EQ(out.x, board.x);
	EXPECT_DOUBLE_EQ(out.y, board.y);
	EXPECT_DOUBLE_EQ(out.theta, board.theta);
}

TEST(OdomScale, LinearScaleStretchesStraightDriveOnly)
{
	ScaledOdometry odom;
	const Scale scale{1.02, 1.0};
	odom.Update(Pose2D{0.0, 0.0, M_PI / 2.0}, scale);
	const Pose2D out = DriveArc(odom, Pose2D{0.0, 0.0, M_PI / 2.0}, scale, 2.0, 0.0, 60);
	EXPECT_NEAR(out.x, 0.0, 1e-9);
	EXPECT_NEAR(out.y, 2.04, 1e-9);
	EXPECT_NEAR(out.theta, M_PI / 2.0, 1e-12);
}

TEST(OdomScale, AngularScaleCorrectsTurnsPastPi)
{
	ScaledOdometry odom;
	const Scale scale{1.0, 0.98};
	odom.Update(Pose2D{}, scale);
	// 두 바퀴: 각도가 ±π 를 여러 번 넘어도 누적 회전이 맞아야 한다
	const Pose2D out = DriveArc(odom, Pose2D{}, scale, 0.0, 4.0 * M_PI, 400);
	EXPECT_NEAR(out.theta, odom_scale::NormalizeAngle(4.0 * M_PI * 0.98), 1e-9);
	EXPECT_NEAR(out.x, 0.0, 1e-12);
}

TEST(OdomScale, ArcKeepsShapeWithScaledHeading)
{
	// 배율 1 과 거의 같으면 다시 적분한 결과도 보드 값(배율 1 은 그대로 통과)과 같아야 한다
	ScaledOdometry odom;
	ScaledOdometry passthrough;
	const Scale scale{1.0, 1.0 + 1e-12};
	odom.Update(Pose2D{}, scale);
	const Pose2D out = DriveArc(odom, Pose2D{}, scale, M_PI, M_PI, 300);  // 반지름 1 m 반원
	const Pose2D board = DriveArc(passthrough, Pose2D{}, Scale{}, M_PI, M_PI, 300);
	EXPECT_NEAR(out.x, board.x, 1e-9);
	EXPECT_NEAR(out.y, board.y, 1e-9);
}

TEST(OdomScale, SwitchingScaleDoesNotJump)
{
	ScaledOdometry odom;
	Pose2D board{0.5, 0.2, 0.3};
	odom.Update(board, Scale{});
	const Pose2D out = odom.Update(board, Scale{1.1, 0.9});
	EXPECT_DOUBLE_EQ(out.x, board.x);
	EXPECT_DOUBLE_EQ(out.y, board.y);
	EXPECT_DOUBLE_EQ(out.theta, board.theta);
}

TEST(OdomScale, FileRoundTripAndRangeCheck)
{
	const std::string path = "/tmp/test_odom_scale_" + std::to_string(::getpid()) + ".txt";
	ASSERT_TRUE(odom_scale::Save(path, Scale{1.0123, 0.9876}));
	Scale loaded;
	ASSERT_TRUE(odom_scale::Load(path, loaded));
	EXPECT_DOUBLE_EQ(loaded.linear, 1.0123);
	EXPECT_DOUBLE_EQ(loaded.angular, 0.9876);
	ASSERT_TRUE(odom_scale::Save(path, Scale{1.5, 1.0}));
	EXPECT_FALSE(odom_scale::Load(path, loaded));
	EXPECT_FALSE(odom_scale::Load(path + ".missing", loaded));
	std::remove(path.c_str());
}
