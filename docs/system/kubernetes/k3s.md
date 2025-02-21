# K3S

K3S is a lightweight distribution of Kubernetes and it's tailored for hardware with limited resources. It's easy to set up on embedded computers and stuiable for use in small-scale applications.

## Install K3S cluster

* Install master

```bash
$ curl -sfL https://get.k3s.io | sh -
```

* Install agent node

```bash
$ curl -sfL https://get.k3s.io | K3S_URL=https://myserver:6443 K3S_TOKEN=mynodetoken sh -

# or if you want to install a specific version 
$ curl -sfL https://get.k3s.io | INSTALL_K3S_VERSION="v1.30.6+k3s1" K3S_URL=https://myserver:6443 K3S_TOKEN=mynodetoken sh -

# you may check version your existing nodes
$ kubectl version
```

Note that you need to update `myserver` to your actual server address. `mynodetoken` can be found at `/var/lib/rancher/k3s/server/node-token` on the server node.

## Uninstall K3S cluster

* Uninstall the master node

```bash
$ /usr/local/bin/k3s-uninstall.sh
```

* Uninstall the agent node

```bash
$ /usr/local/bin/k3s-agent-uninstall.sh
```

## Reset K3S cluster

* For the master node

```bash
$ sudo systemctl stop k3s
$ sudo rm -rf /var/lib/rancher/k3s
$ sudo rm -rf /etc/rancher/k3s
$ sudo systemctl start k3s
```

* For the agent nodes

```bash
$ sudo systemctl stop k3s-agent
$ sudo rm -rf /var/lib/rancher/k3s/agent
$ sudo rm -rf /etc/rancher/k3s/agent
$ sudo systemctl start k3s-agent
```

## Reset the certificates

```bash
$ sudo rm /var/lib/rancher/k3s/server/tls/dynamic-cert.json
$ sudo kubectl --insecure-skip-tls-verify=true delete secret -n kube-system k3s-serving
$ sudo systemctl restart k3s
```

To verify that all K3S internal certificates are valid:

```bash
$ sudo su
$ for i in `ls /var/lib/rancher/k3s/server/tls/*.crt`; do echo $i; openssl x509 -enddate -noout -in $i; done

$ curl -v -k https://localhost:6443
```

## Remove a client node

```bash
$ kubectl drain <node-name>

# you may need to ignore daemonsets and delete local-data
$ kubectl drain <node-name> --ignore-daemonsets --delete-local-data

$ kubectl delete node <node-name>
```

## Purge a namespace

```bash
# delete by type
$ kubectl delete all --all -n mynamespace
$ kubectl delete configmaps --all -n mynamespace
$ kubectl delete secrets --all -n mynamespace
$ kubectl delete pvc --all -n mynamespace
$ kubectl delete ingress --all -n mynamespace
$ kubectl delete namespace mynamespace

# or combine into one command
$ kubectl api-resources --verbs=list --namespaced -o name \
  | xargs -n 1 kubectl delete --all -n mynamespace
```

If the command to delete the namespace get stuck:

```bash
$ kubectl edit namespace <namespace>
```

Remove the finalizers section and save. Then try to delete the namespace again:

```bash
$ kubectl delete namespace arc-runners
```

## Manage images of the cluster

To see what images have been pulled locally

```bash
sudo k3s crictl images 
```

To delete any images no currently used by a running container

```bash
sudo k3s crictl rmi --prune 
```

## Check status of pods

If a pod fails to start, you may get information about the error:

```bash
$ kubectl describe pod <podname> -n <namespace>
```

You may also check logs of the pod


```bash
$ kubectl logs -f <podname>
```

To get an overview of all pods in the cluster:

```bash
$ kubectl get pods --all-namespaces -o wide
```


## Reference

* https://docs.k3s.io/
* https://stackoverflow.com/questions/35757620/how-to-gracefully-remove-a-node-from-kubernetes