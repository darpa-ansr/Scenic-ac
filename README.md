This repository contains a modified version of Scenic that adds a capability to replay ROS bags
collected from Airsim ADK simulations. The replay capability is intended to be used in conjunction
with `tools/ansr/ac_scenario_generator.py` for assurance claim verification.

# Installation
We recommend installing Scenic-ac in a virtual environment to avoid overwriting any existing Scenic
installations. Scenic-ac should be identical to Scenic 3, aside from the replay capability, however
this is a beta release and bugs are possible, if not likely. To install Scenic-ac in a virtual
environment

1. Create and activate a Python3 virtual environment of your choice
2. Clone the Scenic-ac repository and enter with `cd Scenic-ac`
3. Execute `python3 -m pip install .`
4. Test replay simulator with `pytest tests/simulators/replay/test_replay.py`

# Usage
To use replay simulator the scenario file must

1. Set `param sim_data` to Pandas dataframe containing the simulation data (`scenic.simulators.utils.parse_bag.bag_to_dataframe()` should be used for ROS bag to dataframe translation)
2. Include `model scenic.domains.driving.replay`
3. Create simulation objects for each entity in the simulation (`tools/ansr/ac_scenario_generator.py` should
be used for this)

See `tests/simulators/replay/replay.scenic` and `examples/driving/replay/replay.scenic` for examples.

# Assurance Claim Verification
To verify assurance claim holds for a simulation ROS bag recording, use
`tools/ansr/ac_scenario_generator.py` to first generate the matching Scenic scenario file. Scenario
generator takes the ROS bag, mission `description.json` and `config.json`, and a collection of
assurance claim markdown files (in the format described
[here](https://github.com/darpa-ansr/assurance-claims/blob/main/eval02/assurance_claim_formalism.md))
files to generate a collection of scenario files, one per assurance claim file, that will verify the
claim. For example, the following command line can be used in `tools/ansr/` directory

```
python3 ac_scenario_generator.py sim_data sim_data assurance_claims
```

IMPORTANT: The provided paths must be absolute unless you plan to run `scenic` in the same directory
where `ac_scenario_generator.py` was run.

`ac_scenario_generator.py` will create a `.scenic` file for each assurance claim `.md` file in the
same directory as the assurance claim file. So, if the assurance claim directory contains
`test_assurance_claim_1.md` and `test_assurance_claim_2.md`, the result will be files
`test_assurance_claim_1.scenic` and `test_assurance_claim_2.scenic` in the same directory. The
assurance claim predicates are translated into matching require statements in the Scenic scenario
files. The domain of applicability section of the assurance claim file is currently ignored.

The generated Scenic scenario files can now be run in Scenic-ac replay mode to check whether the
assurance claim holds. In the virtual environment where Scenic-ac is installed, from
`tools/ansr`, execute

```
scenic --replay -v2 test_assurance_claim_1.scenic
```

If the assurance claim is satisfied the output will look something like this

```
$ scenic --replay -b -v2 replay_test/test_assurance_claim_1.scenic
Beginning scenario construction...
  Compiling Scenic module from ....
  test_assurance_claim_1.scenic...
bag_path=....
pygame 2.5.2 (SDL 2.28.2, Python 3.10.12)
Hello from the pygame community. https://www.pygame.org/contribute.html
    Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/driving_model.scenic...
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/model.scenic...
      Compiled Scenic module in 0.003621 seconds.
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/domains/driving/replay.scenic...
      Compiled Scenic module in 0.03958 seconds.
    Compiled Scenic module in 0.07994 seconds.
  Compiled Scenic module in 11.44 seconds.
  Pruning scenario...
  Pruned scenario in 1.955e-05 seconds.
Scenario constructed in 11.45 seconds.
  Generated scene in 1 iterations, 0.02389 seconds.
  Beginning simulation of top-level scenario...
  Starting simulation 1...
  Simulation 1 ended successfully at time step 2640 because: termination condition on line 59
  Ran simulation in 11.34 seconds.
```

The next to last line "Simulation 1 ended successfully ..." indicates that no violation occurred.

If the assurance claim is violated at the initial state of the simulation the output will look like

```
Beginning scenario construction...
  Compiling Scenic module from ...
bag_path=...
pygame 2.5.2 (SDL 2.28.2, Python 3.10.12)
Hello from the pygame community. https://www.pygame.org/contribute.html
    Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/driving_model.scenic...
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/model.scenic...
      Compiled Scenic module in 0.003378 seconds.
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/domains/driving/replay.scenic...
      Compiled Scenic module in 0.05574 seconds.
    Compiled Scenic module in 0.07528 seconds.
  Compiled Scenic module in 10.98 seconds.
  Pruning scenario...
  Pruned scenario in 1.597e-05 seconds.
Scenario constructed in 10.99 seconds.
  Rejected sample 1 because of User requirement violation: requirement on line 61
Traceback (most recent call last; use -b to show Scenic internals):
  <Scenic internals>
scenic.core.distributions.RejectionException: failed to generate scenario in 1 iterations
```

If the assurance claim is violated during the simulation the output will look like

```
Beginning scenario construction...
  Compiling Scenic module from ...
bag_path=...
pygame 2.5.2 (SDL 2.28.2, Python 3.10.12)
Hello from the pygame community. https://www.pygame.org/contribute.html
    Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/driving_model.scenic...
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/simulators/replay/model.scenic...
      Compiled Scenic module in 0.003403 seconds.
      Compiling Scenic module from /home/genindi1/projects/ANSR/Scenic/src/scenic/domains/driving/replay.scenic...
      Compiled Scenic module in 0.03797 seconds.
    Compiled Scenic module in 0.05785 seconds.
  Compiled Scenic module in 10.93 seconds.
  Pruning scenario...
  Pruned scenario in 1.907e-05 seconds.
Scenario constructed in 10.95 seconds.
  Generated scene in 1 iterations, 0.008917 seconds.
  Beginning simulation of top-level scenario...
  Starting simulation 1...
  Rejected simulation 1 at time step 96 because: requirement on line 66
  Ran simulation in 0.3905 seconds.
```

The line "Rejected simulation 1 at time ... " will list the line number of the violated require
statement. There will only be one require statement corresponding to the assurance claim.

NOTE: Scenario generated for `tools/ansr/assurance_claims/test_assurance_claim_2.md` by
`ac_scenario_generator.py` causes Scenic to fail with "invalid syntax" error. It appears to be
an issue with Scenic parser that we are currently working to resolve.

# Standard Scenic documentation below

[<img src="https://docs.scenic-lang.org/en/latest/_static/logo-full.svg" alt="Scenic Logo" height="100">](https://scenic-lang.org/)

[![Documentation Status](https://readthedocs.org/projects/scenic-lang/badge/?version=latest)](https://docs.scenic-lang.org/en/latest/?badge=latest)
[![Tests Status](https://github.com/BerkeleyLearnVerify/Scenic/actions/workflows/run-tests.yml/badge.svg)](https://github.com/BerkeleyLearnVerify/Scenic/actions/workflows/run-tests.yml)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A compiler and scenario generator for Scenic, a domain-specific probabilistic programming language for modeling the environments of cyber-physical systems.
Please see the [documentation](https://docs.scenic-lang.org/) for installation instructions, as well as tutorials and other information about the Scenic language, its implementation, and its interfaces to various simulators.

For an overview of the language and some of its applications, see our [2022 journal paper](https://link.springer.com/article/10.1007/s10994-021-06120-5) on Scenic 2, which extends our [PLDI 2019 paper](https://arxiv.org/abs/1809.09310) on Scenic 1.
The new syntax and features of Scenic 3 are described in our [CAV 2023 paper](https://arxiv.org/abs/2307.03325).
Our [Publications](https://docs.scenic-lang.org/en/latest/publications.html) page lists additional relevant publications.

Scenic was initially designed and implemented by Daniel J. Fremont, Tommaso Dreossi, Shromona Ghosh, Xiangyu Yue, Alberto L. Sangiovanni-Vincentelli, and Sanjit A. Seshia.
Additionally, Edward Kim made major contributions to Scenic 2, and Eric Vin, Shun Kashiwa, Matthew Rhea, and Ellen Kalvan to Scenic 3.
Please see our [Credits](https://docs.scenic-lang.org/en/latest/credits.html) page for details and more contributors.

If you have any problems using Scenic, please submit an issue to [our GitHub repository](https://github.com/BerkeleyLearnVerify/Scenic) or start a conversation on our [community forum](https://forum.scenic-lang.org/).

The repository is organized as follows:

* the _src/scenic_ directory contains the package proper;
* the _examples_ directory has many examples of Scenic programs;
* the _assets_ directory contains meshes and other resources used by the examples and tests;
* the _docs_ directory contains the sources for the documentation;
* the _tests_ directory contains tests for the Scenic tool.
