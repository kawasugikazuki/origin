

class MesurementData:
    def __init__(self, group, id, step, height_correction, distance, azimuth, rotation, transit_time, inner_r_th, mu, sigma, outer_r_th, reject_mode, marker_color, decision, collision_for_goal, collision_for_avoidance, until_collision) -> None:

        self.group = group
        self.id = id
        self.step = step
        self.height_correction = height_correction
        self.distance = distance
        self.azimuth = azimuth
        self.rotation = rotation
        self.transit_time = transit_time
        self.inner_r_th = inner_r_th
        self.mu = mu
        self.sigma = sigma
        self.outer_r_th = outer_r_th
        self.reject_mode = reject_mode
        self.marker_color = marker_color
        self.decision = decision #0:accept, 1:reject, 2:boids, 3:avoidance
        self.collision_for_goal = collision_for_goal
        self.collision_for_avoidance = collision_for_avoidance
        self.until_collision = until_collision

    def output_csv_format(self):
        
        csv_format_str = self.group + "," + \
                         self.id + "," + \
                         str(self.step) + "," + \
                         str(self.height_correction) + "," + \
                         str(self.distance) + "," + \
                         str(self.azimuth) + "," + \
                         str(self.rotation) + "," + \
                         str(self.transit_time) + "," + \
                         str(self.inner_r_th) + "," + \
                         str(self.mu) + "," + \
                         str(self.sigma) + "," + \
                         str(self.outer_r_th) + "," + \
                         self.reject_mode + "," + \
                         self.marker_color + "," + \
                         str(self.decision) + "," + \
                         str(self.collision_for_goal) + "," + \
                         str(self.collision_for_avoidance) + "," + \
                         str(self.until_collision)
        
        print(csv_format_str)
        
        return csv_format_str
    
#if __name__ == "__main__":
#    a = MesurementData("RED", "192.168.1.xxx", 1, True, 1.5, 25.3, 3.1, 1, 2, 0.5, 3, "A", "5", 3, 1, 0, 1.3)
#    print(a.output_csv_format())