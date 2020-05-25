class Log:
	def __init__(self):
		self.datasets = {}
		self.time_points = []

	def set_time_source(self, source):
		self.time_source = source

	def add_dataset(self, label, names, sources):
		if ((type(names) == type([]) and type(sources) != type([])) or (type(names) != type([]) and type(sources) == type([]))):
			return # Add error handling here eventually
		elif(len(names) != len(sources)):
			return # And here
		else:
			if(type(sources) == type([])):
				self.datasets[label] = {};
				self.datasets[label]["legend"] = names
				for i in range(0, len(names)):
					name = names[i]
					source = sources[i]
					self.datasets[label][name] = {}
					self.datasets[label][name]["name"] = name
					self.datasets[label][name]["source"] = source
					self.datasets[label][name]["points"] = []

	def sample(self):
		self.time_points.append(self.time_source())
		for dataset in self.datasets:
			for data in self.datasets[dataset]["legend"]:
				self.datasets[dataset][data]["points"].append(self.datasets[dataset][data]["source"]())

	def get_dataset_labels(self):
		return ["time"] + list(self.datasets.keys())

	def get_dataset(self, label):
		if label == "time":
			return self.time_points
		elif label in self.datasets.keys():
			return self.datasets[label]
		else: 
			pass # TODO: error handling


'''class Log:
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

	def add_data_source(self, name, source, label=None):
		if type(source) == type([]) && type(name) == type([]):
			self.data_names.append
			for i in range(0, len(name)):
				self.data
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
'''