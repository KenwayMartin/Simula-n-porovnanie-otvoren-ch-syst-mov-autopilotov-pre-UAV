setsid gz service -s /world/baylands/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/home/user/Projects/BP/simulation/models/wall.sdf", name: "bp_test_wall"' < /dev/null &
pids+=("$!")