class Log:
	def __init__(self):
		self.time_points = []
		self.data_points = []
		self.data_sources = []
		self.data_names = []

	def log_point(self, time, value):
		self.time_points.append(time)
		self.data_points.append(time)

	def set_time_source(self, source):
		self.time_source = source

	def add_data_source(self, name, source):
		self.data_sources.append(source)
		self.data_names.append(name)
		self.data_points.append([])

	def get_data_sets(self):
		return ["time"] + self.data_names

	def sample(self):
		self.time_points.append(self.time_source())
		for i in range(0, len(self.data_sources)):
			self.data_points[i].append(self.data_sources[i]())

	def get_data_set(self, title):
		if title == "time":
			return self.time_points
		else:
			if title in self.data_names:
				idx = self.data_names.index(title)
				return self.data_points[idx]

	def export_csv(self, file):
		f = open(file, "w")
		line = ""
		for title in self.get_data_sets():
			line = line + str(title) + ","
		line = line[:-1] + "\n"
		f.write(line)
		for i in range(0,len(self.time_points)):
			line = ""
			for title in self.get_data_sets():
				line = line + str(self.get_data_set(title)[i]) + ","
			line = line[:-1] + "\n"
			f.write(line)
		f.close()
