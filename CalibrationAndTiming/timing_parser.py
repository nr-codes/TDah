"""
	A note on local variable suffix naming conventions:
	ts = timestamp, s = start, e = end
"""

from math import sqrt
from sys import argv

# header info
HEADER = '\t'.join([
			'input frames per second (fps)',
			'avg. frame grabber frame rate (fps)',
			'avg. performance counter frame rate (fps)',
			'avg. Fg_getLastPicNumberBlocking frame rate (fps)',
			'avg. threshold frame rate (fps)',
			'avg. tracking frame rate (fps)',

			'avg. frame grabber frame time (spf)',
			'std. dev of frame grabber frame time (spf)',
			'avg. overhead of frame grabber frame time (spf)',

			'avg. performance counter frame time (spf)',
			'std. dev of performance counter frame time (spf)',
			'avg. overhead of performance counter frame time (spf)',

			'avg. Fg_getLastPicNumberBlocking frame time (spf)',
			'std. dev of Fg_getLastPicNumberBlocking frame time (spf)',
			'avg. overhead of Fg_getLastPicNumberBlocking frame time (spf)',

			'avg. threshold frame time (spf)',
			'std. dev of threshold frame time (spf)',
			'avg. overhead of threshold frame time (spf)',

			'avg. tracking frame time (spf)',
			'std. dev of tracking frame time (spf)',
			'avg. overhead of tracking frame time (spf)',

			'last image number',
			'next image number',
			'image lag (frames)',
			'dropped images (frames)',
			'test run number',
			'input frame time (s)',
			'input exposure time (s)',
			'input width (pixels)',
			'input height (pixels)'
		])

"""
def stats(x, w):
	Calculates mean and variance
	n = len(x)
	s = 0.0
	ss = 0.0
	ws = 0.0
	wss = 0.0
	
	for i in xrange(n):
		s = x[i] * w[i] + s
		ss = w[i] * x[i] * x[i] + ss
		ws = w[i] + ws
		wss = w[i] * w[i] + wss
	
	m = s / ws
	v = (ss * ws - s * s) / (ws * ws - wss)
	return (m, v)
"""

def stats(x, n):
	s = 0.0
	ss = 0.0

	for i in xrange(len(x)):
		s = x[i] + s
		ss = x[i] * x[i] + ss

	m = s / n
	v = (ss - n * m * m) / (n - 1)
	return (m, v)

def data(params):
	"""Unpack data in table and store in params."""

	(	img, 
		roi, 
		pc_ts,
		fg_ts, 
		gr_s, 
		gr_e, 
		th_s, 
		th_e,
		tr_s, 
		tr_e,
		roi_x, 
		roi_y, 
		roi_w, 
		roi_h,
		blob_x, 
		blob_y, 
		blob_w, 
		blob_h, 
		blob_f
	) = [long(i) for i in l.split()]

	params['img_num'].append(img)
	params['roi'].append(roi)

	params['fg_ts'].append(fg_ts)
	params['pc_ts'].append(pc_ts)

	# check for overflow..only solves 1 overflow occurance
	if gr_e < gr_s:
		params['gr_se'].append(gr_e + params['max_pc_ts'] - gr_s)
	else:
		params['gr_se'].append(gr_e - gr_s)
	
	if th_e < th_s:
		params['th_se'].append(th_e + params['max_pc_ts'] - th_s)
	else:
		params['th_se'].append(th_e - th_s)

	if tr_e < tr_s:
		params['tr_se'].append(tr_e + params['max_pc_ts'] - tr_s)
	else:
		params['tr_se'].append(tr_e - tr_s)

	params['roi_x'].append(roi_x)
	params['roi_y'].append(roi_y)
	params['roi_w'].append(roi_w)
	params['roi_h'].append(roi_h)

	params['blob_x'].append(blob_x)
	params['blob_y'].append(blob_y)
	params['blob_w'].append(blob_w)
	params['blob_h'].append(blob_h)
	params['blob_f'].append(blob_f)

def reset():
	return {
		# general settings
		'run' : -1,
		'pc_freq' : 0L,
		'max_pc_ts' : 0L,
		'max_fg_ts' : 0L,

		'frame' : 0.0,
		'exposure' : 0.0,
		'width' : 0,
		'height' : 0,
		'last_img' : 0,
		'next_img' : 0,
		'num_imgs' : 0,

		'img_num' : [],
		'roi': [],


		# frame grabber data
		'fg_ts' : [],
		'fg_mean' : 0.0,
		'fg_var' : 0.0,

		# performance counter data
		'pc_ts' : [],
		'pc_mean' : 0.0,
		'pc_var' : 0.0,

		# vision system data
		'gr_se' : [],
		'gr_mean' : 0.0,
		'gr_var' : 0.0,

		'th_se' : [],
		'th_mean' : 0.0,
		'th_var' : 0.0,

		'tr_se' : [],
		'tr_mean' : 0.0,
		'tr_var' : 0.0,

		'roi_x': [],
		'roi_y': [],
		'roi_w' : [],
		'roi_h' : [],

		'blob_x' : [],
		'blob_y' : [],
		'blob_w' : [],
		'blob_h' : [],
		'blob_f' : []
	}

def ts_diff(x, max):
	""" Helper function for calculating deltas between timestamps

			x is the list of timestamps.
			max is the maximum timestamp value in case of an overflow.
	"""
	n = len(x)
	diff = []

	for i in xrange(n - 1):
		# detect timestamp overflow...only detects 1 overflow instance
		if x[i] < x[i + 1]:
			diff.append(x[i + 1] - x[i])
		else:
			diff.append(x[i + 1] + max - x[i])

	return diff

def counts_to_sec_stats(mean, var):
	if params[mean] == 0:
		fps = 0
		ft = 0
	else :
		fps = params['pc_freq'] / params[mean]
		ft = 1 / fps
	sd = sqrt(params[var])
	err = ft - (params['frame'] / 1e6)

	return (fps, ft, sd, err)


def output(params):
	num_imgs = params['num_imgs']

	# calc means and variances
	fg_diffs = ts_diff(params['fg_ts'], params['max_fg_ts'])
	params['fg_mean'], params['fg_var'] = stats(fg_diffs, num_imgs)

	pc_diffs = ts_diff(params['pc_ts'], params['max_pc_ts'])
	params['pc_mean'], params['pc_var'] = stats(pc_diffs, num_imgs)

	params['gr_mean'], params['gr_var'] = stats(params['gr_se'], num_imgs)
	params['th_mean'], params['th_var'] = stats(params['th_se'], num_imgs)
	params['tr_mean'], params['tr_var'] = stats(params['tr_se'], num_imgs)


	# calc fps, spf, standard deviation, and error
	input_fps = 1e6 / params['frame']

	fg_fps = 1e6 / params['fg_mean']
	fg_ft = 1 / fg_fps
	fg_sd = sqrt(params['fg_var'])
	fg_err = (params['fg_mean'] - params['frame']) / 1e6

	pc = counts_to_sec_stats("pc_mean", "pc_var")
	gr = counts_to_sec_stats("gr_mean", "gr_var")
	th = counts_to_sec_stats("th_mean", "th_var")
	tr = counts_to_sec_stats("tr_mean", "tr_var")

	# calc other info
	img_lag = params['next_img'] - params['last_img']
	dropped_imgs = params['last_img'] - params['num_imgs']

	s = [
			input_fps,
			fg_fps,
			pc[0],
			gr[0],
			th[0],
			tr[0],

			fg_ft,
			fg_sd,
			fg_err,

			pc[1],
			pc[2],
			pc[3],

			gr[1],
			gr[2],
			gr[3],

			th[1],
			th[2],
			th[3],

			tr[1],
			tr[2],
			tr[3],

			params['last_img'],
			params['next_img'],
			img_lag,
			dropped_imgs,
			params['run'],
			params['frame'],
			params['exposure'],
			params['width'],
			params['height']
		]

	print '\t'.join([str(i) for i in s])

# important lines in file
IGNORE = 4
RUN = 1
RECS = 2
CPS = 11
MAXPC = 12
MAXFG = 13
FVAL = 14
EVAL = 15
WVAL = 16
HVAL = 17
LSTVAL = 7
NXTVAL = 8
SOR = 19

print HEADER

# parameters to parse out
lnum = 0
eor = 0
params = reset()
f = open(argv[1])

for i in xrange(IGNORE):
	f.readline()

for l in f:
	lnum = lnum + 1

	# parse settings 
	if lnum == RUN:
		params['run'] = int(l.split()[2])
	elif lnum == RECS:
		params['num_imgs'] = int(l.split()[3])
		eor = SOR + params['num_imgs']
	elif lnum == CPS:
		params['pc_freq'] = long(l.split()[4])
	elif lnum == MAXPC:
		params['max_pc_ts'] = long(l.split()[4])
	elif lnum == MAXFG:
		params['max_fg_ts'] = long(l.split()[4])
	elif lnum == FVAL:
		params['frame'] = float(l.split()[2])
	elif lnum == EVAL:
		params['exposure'] = float(l.split()[2])
	elif lnum == WVAL:
		params['width'] = int(l.split()[1])
	elif lnum == HVAL:
		params['height'] = int(l.split()[1])
	elif lnum == LSTVAL:
		params['last_img'] = int(l.split()[5])
	elif lnum == NXTVAL:
		params['next_img'] = int(l.split()[5])

	# parse data
	elif lnum >= SOR and lnum < eor:
		data(params)
	
	# output results
	elif lnum == eor:
		assert l == "\r\n", "file in wrong format or non-windows end-of-line."
		output(params)
		lnum = 0
		eor = 0
		params = reset()

f.close()
