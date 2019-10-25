import numpy as np
from matplotlib import pyplot as plt

fontsize = 24
tickfontsize = 20

methods = ['Our\nMethod', 'LS', '$H_{count}$', 'I-RSC']
x = np.arange(len(methods))

data = [
	(2, 1, [32.4, 59.9, 83.2, 84.2], [2.1, 4.9, 6.6, 9.9]),
	(2, 4, [78., 165.9, 289.5, 317.7], [6.1, 18.8, 19.9, 21.4]),
	(1, 1, [163.9, 220.8, 248.3, 317.9], [21.2, 16.8, 31.2, 45.5]),
]

inflect = {
	1: 'One',
	2: 'Two',
	4: 'Four',
}

plural = {
	1: '',
	2: 's',
	4: 's',
}

for num_arms, num_objs, y, err in data:
	plt.rc('ytick', labelsize=tickfontsize)
	plt.figure()
	plt.bar(x, y, yerr=err, capsize=4, color=(0.,0.,.7))
	plt.xticks(x, [m[2 if m == 'I-RSC' and num_objs == 1 else 0:] for m in methods], rotation=0, size=fontsize)
	plt.ylabel('Planning Time (seconds)', size=fontsize)
	plt.title("{} arm{}, {} Object{}".format(inflect[num_arms], plural[num_arms], inflect[num_objs], plural[num_objs]), size=fontsize)
	plt.gcf().subplots_adjust(left=0.18, right=.98, bottom=.18)
	plt.savefig("{}_arm_{}_obj.pdf".format(num_arms, num_objs), transparent=True)

