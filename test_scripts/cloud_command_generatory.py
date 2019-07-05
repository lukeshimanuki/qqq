num_train = 100
n_objs_pack = 1
time_limit = 300 * n_objs_pack
planning_seed = range(5)
loss = 'dql'
algorithm = 'hpn'
domain = 'two_arm_mover'

if n_objs_pack == 8:
    pidxs = [[20000, 20100]]
    pidxs = [[20000, 20020], [20020, 20040], [20040, 20060], [20060, 20080], [20080, 20100]]
    #pidxs = [[20000, 20020], [20020, 20040], [20040, 20060], [20060, 20080], [20080, 20100]]
else:
    pidxs = [[20000, 20100]]
    #pidxs = [[20000, 20050]]


command = "cd /root/qqq ; git pull;  python test_scripts/threaded_test_%s.py" % algorithm
if algorithm != 'hpn' and algorithm != 'greedy_no_gnn':
    train_seed = [1, 4]
    command += " -domain %s -loss %s -num_train %d -n_objs_pack %d -time_limit %d " % (
        domain, loss, num_train, n_objs_pack, time_limit)
else:
    train_seed = [0, 1]
    command += " -domain %s -n_objs_pack %d -time_limit %d " % (
        domain, n_objs_pack, time_limit)


print "Total instances needed", len(planning_seed) * len(pidxs)
commands = []
#for tseed in train_seed:
for pseed in planning_seed:
    for pidx in pidxs:
        new_command = command + "-planner_seed %d -train_seed %d %d -pidxs %d %d" % (pseed, train_seed[0], train_seed[1], pidx[0], pidx[1])
        print new_command
        raw_input('continue?')
