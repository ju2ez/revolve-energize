from xml.etree import ElementTree

import multineat
from pyrevolve.genotype.cppnneat.genotype import CppnneatGenotype
from pyrevolve.genotype.cppnneat_cpg_brain.config import CppnneatCpgBrainConfig
from pyrevolve.revolve_bot.brain import Brain
from pyrevolve.revolve_bot.brain.cpg import BrainCPG
from pyrevolve.revolve_bot.revolve_bot import RevolveBot
from pyrevolve.revolve_bot.revolve_module import CoreModule


def cppnneat_cpg_brain_develop(
    genotype: CppnneatGenotype, config: CppnneatCpgBrainConfig, body: CoreModule
) -> Brain:
    brain = BrainCPG()
    brain.abs_output_bound = config.abs_output_bound
    brain.use_frame_of_reference = config.use_frame_of_reference
    brain.signal_factor_all = config.signal_factor_all
    brain.signal_factor_mid = config.signal_factor_mid
    brain.signal_factor_left_right = config.signal_factor_left_right
    brain.range_lb = config.range_lb
    brain.range_ub = config.range_ub
    brain.init_neuron_state = config.init_neuron_state
    brain.load_brain = config.load_brain
    brain.output_directory = config.output_directory
    brain.run_analytics = config.run_analytics
    brain.reset_robot_position = config.reset_robot_position
    brain.reset_neuron_state_bool = config.reset_neuron_state_bool
    brain.reset_neuron_random = config.reset_neuron_random
    brain.verbose = config.verbose
    brain.startup_time = config.startup_time

    # Convert to sdf so we can extract things like position and order of actuators exactly like they would be read by the plugin
    bot = RevolveBot("dummy")
    bot._body = body
    bot._brain = BrainCPG()  # dummy
    bot.update_substrate()
    sdf = bot.to_sdf()
    root = ElementTree.fromstring(sdf)
    namespaces = {"rv": "https://github.com/ci-group/revolve"}
    actuators = root.findall(
        "model/plugin[@name='robot_controller']/rv:robot_config/rv:brain/rv:actuators/rv:servomotor",
        namespaces,
    )

    # calculate weights from actuators
    brain.weights = []

    # TODO which weights first. connection or internal?

    brain_net = multineat.NeuralNetwork()
    genotype.multineat_genome.BuildPhenotype(brain_net)

    parsecoords = lambda coordsstr: list(map(lambda x: float(x), coordsstr.split(";")))

    for actuator in actuators:
        coords = parsecoords(actuator.attrib["coordinates"])
        brain_net.Input([coords[0], coords[1], coords[2], 0.0, 0.0, 0.0])
        brain_net.Activate()
        weight = brain_net.Output()[0]
        brain.weights.append(weight)

    for i, actuator in enumerate(actuators[:-1]):
        for neighbour in actuators[i + 1 :]:
            leftcoords = parsecoords(actuator.attrib["coordinates"])
            rightcoords = parsecoords(neighbour.attrib["coordinates"])
            if (
                abs(leftcoords[0] - rightcoords[0])
                + abs(leftcoords[1] - rightcoords[1])
                + abs(leftcoords[2] - rightcoords[2])
                < 2.01
            ):
                brain_net.Input(
                    [
                        leftcoords[0],
                        leftcoords[1],
                        leftcoords[2],
                        rightcoords[0],
                        rightcoords[1],
                        rightcoords[2],
                    ]
                )
                brain_net.Activate()
                weight = brain_net.Output()[0]
                brain.weights.append(weight)

    print(len(brain.weights))

    return brain
