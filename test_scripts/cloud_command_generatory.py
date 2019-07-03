num_train = 5000
n_objs_pack = 8
time_limit = 300 * n_objs_pack
planning_seed = range(5)
loss = 'largemargin'
algorithm = 'greedy'
domain = 'two_arm_mover'

if n_objs_pack == 8:
    pidxs = [[20000, 20020], [20020, 20040], [20040, 20060], [20060, 20080], [20080, 20100]]
    pidxs = [[20000, 20050], [20050, 20100]]
    pidxs = [[20000, 20020], [20020, 20040], [20040, 20060], [20060, 20080], [20080, 20100]]
else:
    pidxs = [[20000, 20100]]
    #pidxs = [[20000, 20050]]


command = "cd /root/qqq ; git pull;  python test_scripts/threaded_test_%s.py" % algorithm
if algorithm != 'hpn' and algorithm != 'greedy_no_gnn':
    train_seed = [0]
    command += " -domain %s -loss %s -num_train %d -n_objs_pack %d -time_limit %d " % (
        domain, loss, num_train, n_objs_pack, time_limit)
else:
    train_seed = [0]
    command += " -domain %s -n_objs_pack %d -time_limit %d " % (
        domain, n_objs_pack, time_limit)


print "Total instances needed", len(train_seed) * len(planning_seed) * len(pidxs)
commands = []
for tseed in train_seed:
    for pseed in planning_seed:
        for pidx in pidxs:
            new_command = command + "-planner_seed %d -train_seed %d -pidxs %d %d" % (pseed, tseed, pidx[0], pidx[1])
            print new_command
            raw_input('continue?')
