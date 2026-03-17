# compute_correction.py

# change to minus if a coordinate is in opposite
X_SIGN = 1.0
Y_SIGN = 1.0

def clamp(value, low, high):
    return max(low, min(high, value))


class CorrectionComputer:
    def __init__(
            self,
            # the middle 10%x10% plate
            deadband_x=0.10,
            deadband_y=0.10,
            kp_x=0.35,
            kp_y=0.35,
            max_vx=0.5,
            max_vy=0.5,
    ):
        self.deadband_x = float(deadband_x)
        self.deadband_y = float(deadband_y)
        self.kp_x = float(kp_x)
        self.kp_y = float(kp_y)
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)

    def compute_align_command(self, latest_msg, height_m):
        """
        Convert normalized image offset (dxn, dyn) into body-frame velocity command.

        Returns dict:
        {
            "valid": bool,
            "vx": float,
            "vy": float,
            "vz": float,
            "aligned": bool
        }

        For this first high-altitude version:
        - keep altitude unchanged -> vz = 0
        - use dxn/dyn only
        - height_m is reserved for future geometric scaling
        """
        result = {
            "valid": False,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "aligned": False,
        }

        if latest_msg is None:
            return result

        if latest_msg.get("det", 0) != 1:
            return result

        dxn = float(latest_msg.get("dxn", 0.0))
        dyn = float(latest_msg.get("dyn", 0.0))

        # Check whether already inside the center 10% x 10% box
        x_in_band = abs(dxn) <= self.deadband_x
        y_in_band = abs(dyn) <= self.deadband_y

        if x_in_band and y_in_band:
            result["valid"] = True
            result["aligned"] = True
            return result

        # Apply deadband axis by axis
        if abs(dxn) <= self.deadband_x:
            dxn = 0.0
        if abs(dyn) <= self.deadband_y:
            dyn = 0.0

        # Map image error -> body-frame velocity
        # Keep signs configurable because real flight test may show one axis is reversed
        vy = Y_SIGN * self.kp_y * dxn
        vx = X_SIGN * self.kp_x * dyn

        # Clamp to speed limits for high-altitude align
        vx = clamp(vx, -self.max_vx, self.max_vx)
        vy = clamp(vy, -self.max_vy, self.max_vy)

        result["valid"] = True
        result["vx"] = vx
        result["vy"] = vy
        result["vz"] = 0.0
        result["aligned"] = False
        return result