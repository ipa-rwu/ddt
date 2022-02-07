# Set up a cluster

## control plane

* Install
https://kubernetes.io/docs/setup/production-environment/tools/kubeadm/install-kubeadm/

* init
 https://kubernetes.io/docs/setup/production-environment/tools/kubeadm/create-cluster-kubeadm/

* setup network
[flannel](https://raw.githubusercontent.com/flannel-io/flannel/master/Documentation/kube-flannel.yml)

* Create limited resources access service account
  * Create the pod-viewer-role.yaml file with the following content:
    ```
    kind: ClusterRole
    apiVersion: rbac.authorization.k8s.io/v1
    metadata:
    name: pod-viewer
    rules:
    - apiGroups: [""] # core API group
    resources: ["pods", "namespaces"]
    verbs: ["get", "watch", "list"]
    ```
  * Create the service account associated with this role:
    ```
    kubectl create serviceaccount pod-viewer
    kubectl apply -f pod-viewer-role.yaml
    kubectl create clusterrolebinding pod-viewer-sa \
    --clusterrole=pod-viewer \
    --serviceaccount=default:pod-viewer-sa
    ```
  * Copy the token from the generated secret
    ```
    kubectl get secret | grep pod-viewer
    ```
  * login inside a particular container
    ```
    kubectl exec -it <pod-name> -n <namespace> -- bash
    ```
* Dashboard
  * Deploying the Dashboard UI
  ```
  kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/v2.4.0/aio/deploy/recommended.yaml
  ```

  * access to the Dashboard
  ```
  kubectl proxy
  ```
* Merge Multiple YAML Files Into One
  * create a kustomization.yaml
    ```
    apiVersion: kustomize.config.k8s.io/v1beta1
    kind: Kustomization

    resources:
      - services.yaml
      - deployments.yaml
      - namespaces.yaml
      - storage.yaml
    ```
  * merge
    ```
    kubectl kustomize build "folder_name" > compiled.yaml
    ```
