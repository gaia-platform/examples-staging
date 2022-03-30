# examples-internal

This is where example applications belong that are not also part of the [core examples](https://github.com/gaia-platform/GaiaPlatform/tree/master/production/examples) included with with the SDK debian install package.

The purpose of examples-internal is to provide a private repo for submitting and internally reviewing examples before deploying them to the public facing [examples repo](https://github.com/gaia-platform/examples). This helps keep the commit history cleaner in the public repo.

The example applications here do not necessarily need to be deployed to the public facing examples repo, unless there is a customer who needs access to a particular example or if an example is to be deployed to the sandbox which pulls source code from the public examples repo.

## Deploy to examples public repo
To deploy a new example or new version of an existing example to the public examples repo:
- Ensure that your new or updated example has been properly reviewed, approved and merged here in examples-internal (or in [GaiaPlatform](https://github.com/gaia-platform/GaiaPlatform/tree/master/production/examples) for the core examples).
- Create a branch with an appropriate name in the examples repo.
- Copy the new and updated files into the corresponding same place in your new branch.
- Push your branch and submit a PR in the public examples repo.
- Post to the engineering slack channel that you have submitted a public examples PR that was previously approved internally.

To deploy an example from the [core examples](https://github.com/gaia-platform/GaiaPlatform/tree/master/production/examples) to the public examples repo follow the same procedure as above. In general the only reason to deploy a core example to the public examples repo is if that example is intended to be deployed in the sandbox. Note that any such core example does not also need to be duplicated here in examples-internal.
