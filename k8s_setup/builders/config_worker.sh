MASTET_NAME="master-node"

function label_worker {
  local workers

  workers=$(get_all_nodes)

  IFS=$'\n' list=($workers)
  for i in "${list[@]}"; do
    kubectl label nodes $i machine=$i
  done

}

function get_all_nodes {
  kubectl get nodes -o wide --no-headers | awk '{ print $1}'
}

label_worker
