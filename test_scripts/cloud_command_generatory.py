num_train = 5000
n_objs_pack = 1
time_limit = 300 * n_objs_pack
planning_seed = range(5)
pidxs = [[20000, 20050], [20050, 20100]]
pidxs = [[0, 50], [50, 100]]

loss = 'largemargin'
algorithm = 'mcts'

if algorithm != 'hpn':
    train_seed = [0, 1, 2, 3, 4]
else:
    train_seed = [0]

command = "cd /root/qqq ; git pull;  python test_scripts/threaded_test_%s.py" % algorithm
command += " -loss %s -num_train %d -n_objs_pack %d -time_limit %d " % (loss, num_train, n_objs_pack, time_limit)

commands = []
for tseed in train_seed:
    for pseed in planning_seed:
        for pidx in pidxs:
            new_command = command + "-planner_seed %d -train_seed %d -pidxs %d %d" % (pseed, tseed, pidx[0], pidx[1])
            print new_command
            raw_input('continue?')



