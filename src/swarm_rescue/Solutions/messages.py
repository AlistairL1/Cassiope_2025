class DroneMessage:
    class Category:
        MAPPING = "MAPPING"
        PASS = "PASS"
        LOCK_WOUNDED = "LOCK_WOUNDED"

    def __init__(self, category, arg, drone_id=None):
        self.category = category
        self.arg = arg
        self.drone_id = drone_id