import 'dart:math';

class VehicleState {
  int timestampMs;
  double x, y, theta, v, w, covTrace;

  VehicleState({
    required this.timestampMs,
    required this.x,
    required this.y,
    required this.theta,
    required this.v,
    required this.w,
    this.covTrace = 0.0
  });

  VehicleState projectFuture(double dtSeconds) {
    // Because the stationary car has v=0 and w=0, nextX and nextY will
    // mathematically perfectly equal x and y. No jitter!
    double nextX, nextY, nextTheta = theta + (w * dtSeconds);

    if (w.abs() < 0.001) {
      nextX = x + v * cos(theta) * dtSeconds;
      nextY = y + v * sin(theta) * dtSeconds;
    } else {
      nextX = x + (v / w) * (sin(theta + w * dtSeconds) - sin(theta));
      nextY = y - (v / w) * (cos(theta + w * dtSeconds) - cos(theta));
    }

    return VehicleState(
      timestampMs: timestampMs + (dtSeconds * 1000).toInt(),
      x: nextX, y: nextY, theta: nextTheta, v: v, w: w, covTrace: covTrace
    );
  }
}

class CollisionEngine {
  static const double predictionHorizonS = 2.0;
  static const double predictionStepS = 0.1;

  // Uncertainty expansion. As the prediction looks further into the future,
  // the hitbox grows slightly to account for sensor drift.
  static const double radiusGrowthRate = 0.05;

  VehicleState? latestSelf;
  VehicleState? latestOther;

  // CHANGED: Now accepts the dynamic boundary radius from your UI textbox!
  // Defaults to 0.30m (0.15m per car for a 26x15cm chassis)
  Map<String, dynamic> processAndCheck(VehicleState self, VehicleState other, {double dynamicCollisionBoundary = 0.30}) {

    // 1. Time Synchronization
    int timeDiffMs = self.timestampMs - other.timestampMs;
    if (timeDiffMs > 0) {
      other = other.projectFuture(timeDiffMs / 1000.0);
    } else if (timeDiffMs < 0) {
      self = self.projectFuture(timeDiffMs.abs() / 1000.0);
    }

    // 2. Current Relative Distances
    double dx = other.x - self.x;
    double dy = other.y - self.y;
    double distance = sqrt((dx * dx) + (dy * dy));

    // 3. Radar UI Projections
    double relX = dx * cos(-self.theta) - dy * sin(-self.theta);
    double relY = dx * sin(-self.theta) + dy * cos(-self.theta);
    double relTheta = other.theta - self.theta;

    // 4. Future Trajectory Simulation (The "Look-Ahead" Logic)
    bool isCollision = false;
    double minTTC = 999.0;
    int steps = (predictionHorizonS / predictionStepS).round();

    for (int i = 1; i <= steps; i++) {
      double t = i * predictionStepS;

      VehicleState futureSelf = self.projectFuture(t);
      VehicleState futureOther = other.projectFuture(t); // Will stay perfectly still for stationary car

      double fDx = futureOther.x - futureSelf.x;
      double fDy = futureOther.y - futureSelf.y;
      double futureDistanceSq = (fDx * fDx) + (fDy * fDy);

      // The safe threshold uses the dynamic input, plus the uncertainty growth over time
      double currentThreshold = dynamicCollisionBoundary + (radiusGrowthRate * t);

      // Check if the future trajectory breaches your dynamic safe zone
      if (futureDistanceSq <= (currentThreshold * currentThreshold)) {
        isCollision = true;
        minTTC = t;
        break; // Stop checking further once a collision is predicted
      }
    }

    return {
      'collision': isCollision,
      'ttc': minTTC,
      'distance': distance,
      'relX': relX,
      'relY': relY,
      'relTheta': relTheta,
      'selfV': self.v,
      'otherV': other.v // Will be exactly 0.0 for the stationary car
    };
  }
}