source "./install_k8s.sh"

CLUSTER_NAME="ddt_kubernetes"

function remove_port {
    local ports=("$@")
    for i in "${ports[@]}"; do
        sudo kill -9 `sudo lsof -t -i:$ports`
    done
}

function clean_up_k8s {
    local cluster_name="$1"

    echo
    echo "Clean up k8s ..."

    kubectl config delete-cluster "$cluster_name"
    sudo kubeadm reset
    sudo rm $HOME/.kube/config
}

function create_default_kubeadm_config {
    local cluster_name=$1
    local temp_config
    temp_config=$(mktemp -t kubeadm.XXXXXXXX.yaml)
  cat > "${temp_config}" << EOF
apiVersion: kubeadm.k8s.io/v1beta3
kind: ClusterConfiguration
kubernetesVersion: 1.22.0
networking:
  podSubnet: 10.244.0.0/16
clusterName: "${cluster_name}"
EOF
  echo "${temp_config}"

}

function init_kubeadm {
  local kubeadm_yaml=$1
  sudo kubeadm init --config "${kubeadm_yaml}"
}

function setup_cluster {
  local cluster_name=$1
  local kubeadm_yaml=$2

  # This world-readable file is created by "kubeadm init" (and deleted by "kubeadm reset"),
  # so we use it to detect an existing local cluster.
  if [[ -f /etc/kubernetes/pki/ca.crt ]] ; then
    clean_up_k8s $cluster_name
  fi

  disable_swap

  echo
  echo "Initializing the local cluster..."
  # Retry pulling images as GCR sometimes returns errors, and kubeadm doesn't retry.
  retry sudo kubeadm config images pull --config "${kubeadm_yaml}"
  local kubeadm_results
  kubeadm_results=$(init_kubeadm $kubeadm_yaml)
  local temp_results
  temp_results=$(mktemp -t kubeadm_init.XXXXXXXX.txt)
  cat > "${temp_results}" << EOF
${kubeadm_results}
EOF
  if [[ ! -d ~/.kube/ ]] ; then
    mkdir ~/.kube
  fi
  find ~/.kube/ -maxdepth 1 -name "kubeadm_init.*.txt" -type f -delete
  mv "$temp_results" ~/.kube/

  # Merge generated kubeconfig into ~/.kube/config
  echo
  echo "Adding the local cluster to ~/.kube/config..."
  tmp=$(mktemp)
  # shellcheck disable=2024
  # sudo is required for kubectl to read admin.conf, not to write to $tmp.
  sudo KUBECONFIG=/etc/kubernetes/admin.conf:$HOME/.kube/config \
    kubectl config view --flatten > $tmp
  mkdir --parents ~/.kube
  mv "$tmp" ~/.kube/config
  chmod 600 ~/.kube/config

  # TODO(b/118584427): remove this permissive binding and use proper auth
  echo
  echo "Creating a permissive policy for the local cluster..."
  kubectl create clusterrolebinding permissive-binding \
    --clusterrole=cluster-admin \
    --user=admin \
    --user=kubelet \
    --group=system:serviceaccounts
}

function install_k8s_master {
  local cluster_name=$1
  local kubeadm_yaml=$2

  check_distribution_is_supported

  install_common_deps
  install_docker_deps
  install_k8s_master_deps

  # Setup and configure docker and k8s.
  setup_docker
  setup_k8s
  setup_cluster "${cluster_name}" "${kubeadm_yaml}"

  echo
  echo "The local Kubernetes cluster has been installed."
}

function setup_network {
# use flannel
kubectl apply -f https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml
kubectl apply -f https://raw.githubusercontent.com/k8snetworkplumbingwg/multus-cni/master/deployments/multus-daemonset-thick-plugin.yml
# https://github.com/k8snetworkplumbingwg/multus-cni
}

function main {
  if [[ "$EUID" -eq 1 ]]; then
    echo "Don't run the script as root. Instead use the user-account that will manage the cluster."
    exit 1
  fi
  if [[ "$#" -gt 2 ]]; then
    echo "Usage: $0 [<kubernetes cluster name>]" >&2
    echo "Usage: $1 [<path to kubeadm config (kubeadm.yaml)>]" >&2
    exit 1
  fi

  local cluster_name=$1
  local kubeadm_yaml=$2

  set -eu

  if [[ -z "${cluster_name}" ]] ; then
    cluster_name=$CLUSTER_NAME
  fi
    echo "Using cluster name: $cluster_name"

  if [[ -z "${kubeadm_yaml}" ]] ; then
    echo "kubeadm using temp_config"
    kubeadm_yaml=$(create_default_kubeadm_config $cluster_name)
    # shellcheck disable=2064
    # ${kubeadm_yaml} is intentionally expanded now, rather than when the trap
    # is executed.
    trap "rm -f '${kubeadm_yaml}'" EXIT
    cat ${kubeadm_yaml}
  fi

  install_k8s_master "${cluster_name}" "${kubeadm_yaml}"
  kubectl -n kube-system get cm kubeadm-config -o yaml

  setup_network
}

if (($# == 0)); then
  exit 0
fi

if (($# == 1)); then
  main $1
  exit 0
fi

if (($# == 2)); then
  if [[ $1 == *.yaml || $1 == *.yml ]]; then
    config=$1
    cluster_name=$2
  else
    cluster_name=$1
    config=$2
  fi
  main $cluster_name $config
  exit 0
fi
