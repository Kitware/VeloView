# Continuous Integration

[![pipeline status](https://gitlab.kitware.com/bjacquet/VeloView-kwinternal/badges/master/pipeline.svg)](https://gitlab.kitware.com/bjacquet/VeloView-kwinternal/commits/master)

## Important note

Each time a commit is pushed to gitlab, the following steps happen:
- gitlab checks using the `.gitlab-ci` file which [Jobs] should be run and distributes then to differents [Runners]
- once a [Runners] receives a job to excecute, it will create a clone of a predefined Virtual Machine, and start it.
- because we don't want the superbuild to be rebuild for each job a cache mecanism is used:
    - the superbuild caches are stored on a minio server
    - the runner will try to find the best superbuild for the committed branch
    - **The best superbuild is found given the folowing criteria:**
        - fisrt: the branch specific superbuild
        - then: a parent/master branch specific superbuild
        - finally: the kitware-master branch superbuild

#### example of how the best superbuild is found:

Suppose a commit is pushed on the branch named feature/Slam/.../test/Nick
- first:
  - ---> feature/Slam/LoopClosure/test/Nick
- then:
  - ---> feature/Slam/LoopClosure/test/master
  - ---> feature/Slam/LoopClosure/master
  - ---> feature/Slam/master
  - ---> feature/master
- finally:
  - ---> kitware-master

## Overview of the different tools that are used

### [GitLab CI/CD] to manage all compilation jobs and pipelines

GitLab CI/CD is the fully integrated continuous integration service offered by Gitlab.


A `.gitlab-ci.yml` file describe an [Integration Pipeline] which describe different [Jobs], in our case:
- a **superbuild** job per OS
- a (veloview) **build** job per OS
- a **test** job per OS
- a (superbuild) **package** job per OS
- ...

Each [Jobs] will be executed by different [Runners], in our case:
- Linux VMs
- Windows VMs # currently not available
- Mac VMs # currently not available

Each machine self announces its configured [Runners] to Gitlab through gitlab-runner.

### [Vagrant] to create Virtual Machine

Vagrant is a tool to automatically create virtual machines from scratch and configures it. This make the process reproductible.

### [Minio] to create a cache server

A Minio server is used to cache, for each branch, the Superbuild build folder for each VMs/OS. This avoid to rebuild the Superbuild for each commit. New Superbuild "flavours" can be cached manually if needed.

## Example how to set up everything from scratch

### Manually create a new VM for a new Runner

***dependencie:*** vagrant, virtualbox

```bash
sudo apt install vagrant
```
A `Vagrantfile` fully describe a virtual machine (name, os, memory, cpu, ...), if you need to instantiate a new machine you can directly or based your new VM on `Vagrantfile.<os>`. Also the VM must have installed:
- **a ssh server**
- **a minio client** in order to get parents superbuild cache, if no new superbuild for the current branch is available/needed
- **gitlab-runner** to enable gilab-ci [cache](https://docs.gitlab.com/ee/ci/yaml/#cache) and [artifacts](https://docs.gitlab.com/ee/ci/yaml/#artifacts) function
- **a zip utlity** to unzip the superbuild

Once the 'Vagrantfile' correctly configure, you can instantiate a new VM.

```bash
cd <path/to/Vagrantfile>
vagrant up
```

### Create a new Runner

***dependencie:*** gilab-runner

```bash
curl -L https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh | sudo bash
sudo apt-get install gitlab-runner
```
> It's very important to set **builds_dir** and **cache_dir**, otherwise you would have a weird error like ***Skipping cache extraction due to empty cache key***

The [Runners] are describe in the `.gitlab-runner/config.toml` file
KEU one can be found on Wheezy/Project/VeloView/Internal/CI/.gitlab-runner/config.toml

```bash
concurrent = 1
check_interval = 0

[[runners]]
  name = "<name in gitlab.com>"
  url = "https://gitlab.kitware.com/"
  token = "<>"
  executor = "virtualbox"
  builds_dir = "/home/vagrant/builds"
  cache_dir = "/home/vagrant/caches"
  [runners.ssh]
    user = "vagrant"
    identity_file = "<path/to/Vagrantfile>/.vagrant/machines/default/virtualbox/private_key"
  [runners.virtualbox]
    base_name = "<VM name>"
    disable_snapshots = false
  [runners.cache]
    Type = "s3"
    ServerAddress = "<>"
    AccessKey = "<>"
    SecretKey = "<>"
    BucketName = "runner"
    Insecure = true
    Path = "minio"
    Shared = true
```
Then you need to add the machine [tags](https://docs.gitlab.com/ee/ci/yaml/#tags) via  [Runners settings](https://gitlab.kitware.com/bjacquet/VeloView-kwinternal/settings/ci_cd) in section 

### Create a new Minio server

If you need to setup a new server please refer to [this link](https://docs.gitlab.com/runner/install/registry_and_cache_servers.html#install-your-own-cache-server).


[GitLab CI/CD]: https://docs.gitlab.com/ee/ci/
[Integration Pipeline]: https://docs.gitlab.com/ee/ci/pipelines.html#pipelines
[Jobs]: https://docs.gitlab.com/ee/ci/pipelines.html#jobs
[Runners]: https://docs.gitlab.com/ee/ci/runners/README.html
[Vagrant]: https://www.vagrantup.com/
[Minio]: https://www.minio.io/
