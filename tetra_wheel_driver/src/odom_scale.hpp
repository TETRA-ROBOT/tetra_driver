// 바퀴 오도메트리 보정 배율 (웹 관제: 관리자 설정 → 바퀴 오도메트리 보정)
//
// 오도메트리(x, y, θ)는 구동보드가 엔코더와 보드 안의 바퀴 지름·간격으로 계산해 보낸다. 보드 파라미터 번호의 뜻은
// 문서가 없어서 건드리지 않고, 드라이버가 보드 오도메트리의 매 주기 이동량에 배율을 곱해 다시 적분한다.
//   직진 배율 = 실제 이동 거리 / 오도메트리 거리   (바퀴 지름 오차, 타이어 마모)
//   회전 배율 = 실제 회전 각도 / 오도메트리 각도   (바퀴 간격 오차. 지름 오차도 함께 들어 있다)
// 배율은 로봇마다 달라 ~/.config/tetra 에 둔다 (IMU 자이로 영점과 같은 곳).
#pragma once

#include <cmath>
#include <fstream>
#include <string>

namespace odom_scale
{
// 이보다 크게 틀리면 보정할 일이 아니라 바퀴·엔코더·보드 설정 문제다
constexpr double kMinScale = 0.8;
constexpr double kMaxScale = 1.2;

struct Scale
{
	double linear = 1.0;
	double angular = 1.0;

	bool IsIdentity() const { return linear == 1.0 && angular == 1.0; }
};

inline bool IsValidScale(double value)
{
	return std::isfinite(value) && value >= kMinScale && value <= kMaxScale;
}

inline double NormalizeAngle(double angle)
{
	return std::atan2(std::sin(angle), std::cos(angle));
}

// 파일이 없거나 값이 범위를 벗어나면 false (배율 1 로 시작한다)
inline bool Load(const std::string & path, Scale & scale)
{
	std::ifstream file(path);
	Scale loaded;
	if(!(file >> loaded.linear >> loaded.angular)) return false;
	if(!IsValidScale(loaded.linear) || !IsValidScale(loaded.angular)) return false;
	scale = loaded;
	return true;
}

inline bool Save(const std::string & path, const Scale & scale)
{
	std::ofstream file(path, std::ios::trunc);
	if(!file) return false;
	file.precision(9);
	file << scale.linear << " " << scale.angular << "\n";
	return static_cast<bool>(file);
}

struct Pose2D
{
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
};

// 보드 오도메트리를 받아 배율을 곱한 오도메트리를 낸다
class ScaledOdometry
{
public:
	// 보드가 0 으로 돌아갔을 때(오도메트리 리셋, 재연결) 같이 맞춘다
	void Reset(const Pose2D & board)
	{
		board_prev_ = board;
		scaled_ = board;
		started_ = true;
		diverged_ = false;
	}

	Pose2D Update(const Pose2D & board, const Scale & scale)
	{
		// 배율 1 이고 한 번도 보정한 적이 없으면 보드 값을 그대로 낸다 (예전과 같다). 기준을 계속 옮겨 두어 배율을 켜도 튀지 않는다.
		// 한 번이라도 보정했으면 배율 1 로 돌아와도 보드 값으로 되돌리지 않고 이동량을 계속 더한다.
		// 되돌리면 그동안 쌓인 차이(예: 1.05 로 50 m → 약 2.5 m)만큼 /odom 이 순간 이동한다.
		if(!started_ || (scale.IsIdentity() && !diverged_))
		{
			Reset(board);
			return scaled_;
		}
		if(!scale.IsIdentity()) diverged_ = true;
		const double dx = board.x - board_prev_.x;
		const double dy = board.y - board_prev_.y;
		const double dtheta = NormalizeAngle(board.theta - board_prev_.theta);
		// 보드 좌표의 이동을 로봇 기준(앞·옆)으로 바꿔 배율을 곱한 뒤 보정된 방향으로 다시 놓는다
		const double board_mid = board_prev_.theta + dtheta / 2.0;
		const double forward = (std::cos(board_mid) * dx + std::sin(board_mid) * dy) * scale.linear;
		const double lateral = (-std::sin(board_mid) * dx + std::cos(board_mid) * dy) * scale.linear;
		const double scaled_dtheta = dtheta * scale.angular;
		const double scaled_mid = scaled_.theta + scaled_dtheta / 2.0;
		scaled_.x += std::cos(scaled_mid) * forward - std::sin(scaled_mid) * lateral;
		scaled_.y += std::sin(scaled_mid) * forward + std::cos(scaled_mid) * lateral;
		scaled_.theta = NormalizeAngle(scaled_.theta + scaled_dtheta);
		board_prev_ = board;
		return scaled_;
	}

private:
	Pose2D board_prev_;
	Pose2D scaled_;
	bool started_ = false;
	// 출력이 보드 값과 달라졌는가 (배율 1 이 아닌 값으로 적분한 적이 있음). Reset 하면 다시 같아진다
	bool diverged_ = false;
};
}  // namespace odom_scale
