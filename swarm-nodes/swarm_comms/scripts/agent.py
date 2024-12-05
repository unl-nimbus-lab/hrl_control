class Agent:
    def __init__(self, node_id, responses):
        self.node_id = node_id
        self.responses = responses # intended to be list of tuples of length 3. responses[0][0] = node_id, responses [0][1] = data, responses[0][2] = time elapsed to respond
        self.average_response_time = 0
        self.z = 0
        self.y = 0
        self.x = 0
        self.last_received = 0
        self.current_received = 0
        self.time_between_locations = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.ix = 0
        self.iy = 0
        self.iz = 0

    def __str__(self):
        ret_str = "id:"+str(self.node_id)
        ret_str +=" alt:"+str(self.altitude)
        ret_str +=" lat:"+str(self.latitude)
        ret_str +=" lon:"+str(self.longitude)
        ret_str +=" last_rcvd:"+str(self.last_received)
        ret_str +=" curr_rcvd:"+str(self.current_received)
        ret_str +=" delta_rcvd:"+str(self.time_between_locations)

        return ret_str

    def stats(self):
        ret_str = "Agent "+str(self.node_id)
        ret_str +=": Update delay: " + str(self.time_between_locations) +"ms"
        return ret_str
