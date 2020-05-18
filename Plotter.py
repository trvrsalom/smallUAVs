import pyqtgraph as pg

class Plotter():
	def __init__(self, log, app):
		self.app = pg.QtGui.QApplication([])
		self.window = pg.GraphicsLayoutWidget()
		self.window.setBackground("w")
		self.window.setWindowTitle("Plotter")
		self.plots_per_row = 3
		self.window.setGeometry(750, 0, self.plots_per_row*300, 750)
		self.window.show()
		self.window.raise_()
		self.log = log
		self.init_plots()
		self.domain = 20

	def init_plots(self):
		self.time_log = self.log.get_data_set("time")
		self.plots = {}
		self.plot_items = {}
		plot_titles = self.log.get_data_sets()
		row_ct = 0
		col_ct = 0
		for title in plot_titles:
			if title is "time": continue
			plot_data = self.log.get_data_set(title)
			row = int(row_ct/self.plots_per_row)
			col = col_ct % self.plots_per_row
			plot = self.window.addPlot(row=row, col=col, title=title)
			plot_item = pg.PlotDataItem(self.time_log, plot_data, pen={"width": 1, "color": "b"})
			plot.addItem(plot_item)
			row_ct = row_ct + 1
			col_ct = col_ct + 1
			self.plots[title] = plot
			self.plot_items[title] = plot_item

	def update_plots(self):
		self.time_log = self.log.get_data_set("time")
		plot_titles = self.log.get_data_sets()
		for title in plot_titles:
			if title is "time": continue
			plot_data = self.log.get_data_set(title)
			self.plot_items[title].setData(x=self.time_log, y=plot_data)

	def quit(self):
		self.app.quit()