num_train = 5000
n_objs_pack = 3
time_limit = 300 * n_objs_pack
planning_seed = [0, 1, 2]
train_seed = [1, 2]
pidxs = [[0, 50], [50, 100]]
loss = 'largemargin'

command = "cd /root/qqq ; git pull;  python test_scripts/threaded_test_greedy.py"
command += " -loss %s -num_train %d -n_objs_pack %d -time_limit %d " % (loss, num_train, n_objs_pack, time_limit)

commands = []
for pseed in planning_seed:
    for tseed in train_seed:
        for pidx in pidxs:
            new_command = command + "-planner_seed %d -train_seed %d -pidxs %d %d" % (pseed, tseed, pidx[0], pidx[1])
            print new_command
            raw_input('continue?')



