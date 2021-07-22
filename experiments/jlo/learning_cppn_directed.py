#!/usr/bin/env python3
import asyncio
from dataclasses import dataclass

import multineat
from pyrevolve import parser
from pyrevolve.custom_logging.logger import logger
from pyrevolve.evolution import fitness
from pyrevolve.evolution.pop_management.steady_state import (
    steady_state_population_management,
)
from pyrevolve.evolution.population import Population, PopulationConfig
from pyrevolve.evolution.selection import multiple_selection, tournament_selection
from pyrevolve.experiment_management import ExperimentManagement
from pyrevolve.genotype.bodybrain_composition.config import BodybrainCompositionConfig
from pyrevolve.genotype.bodybrain_composition.crossover import (
    bodybrain_composition_crossover,
)
from pyrevolve.genotype.bodybrain_composition.genotype import (
    BodybrainCompositionGenotype,
)
from pyrevolve.genotype.bodybrain_composition.mutation import (
    bodybrain_composition_mutate,
)
from pyrevolve.genotype.multineat_body.crossover import multineat_body_crossover
from pyrevolve.genotype.multineat_body.develop import multineat_body_develop
from pyrevolve.genotype.multineat_body.genotype import MultineatBodyGenotype
from pyrevolve.genotype.multineat_body.mutation import multineat_body_mutate
from pyrevolve.genotype.multineat_cpg_brain.config import MultineatCpgBrainConfig
from pyrevolve.genotype.multineat_cpg_brain.crossover import (
    multineat_cpg_brain_crossover,
)
from pyrevolve.genotype.multineat_cpg_brain.develop import multineat_cpg_brain_develop
from pyrevolve.genotype.multineat_cpg_brain.genotype import MultineatCpgBrainGenotype
from pyrevolve.genotype.multineat_cpg_brain.mutation import multineat_cpg_brain_mutate
from pyrevolve.util.supervisor.analyzer_queue import AnalyzerQueue
from pyrevolve.util.supervisor.simulator_queue import SimulatorQueue


@dataclass
class GenotypeConstructorConfig:
    bodybrain_composition_config: BodybrainCompositionConfig
    body_multineat_params: multineat.Parameters
    brain_multineat_params: multineat.Parameters


def create_random_genotype(
    config: GenotypeConstructorConfig, id: int
) -> BodybrainCompositionGenotype:
    return BodybrainCompositionGenotype(
        id,
        config.bodybrain_composition_config,
        MultineatBodyGenotype.random(config.body_multineat_params),
        MultineatCpgBrainGenotype.random(config.brain_multineat_params),
    )


async def run():
    """
    The main coroutine, which is started below.
    """

    # experiment settings
    num_generations = 3
    population_size = 30
    offspring_size = 15

    # config for brain development from multineat
    brain_config = MultineatCpgBrainConfig(
        abs_output_bound=1.0,
        use_frame_of_reference=False,
        signal_factor_all=4.0,
        signal_factor_mid=2.5,
        signal_factor_left_right=2.5,
        range_lb=None,
        range_ub=1.0,
        init_neuron_state=0.707,
        load_brain=None,
        output_directory=None,
        run_analytics=None,
        reset_robot_position=None,
        reset_neuron_state_bool=None,
        reset_neuron_random=False,
        verbose=None,
        startup_time=None,
    )

    # body multineat settings
    body_multineat_params = multineat.Parameters()

    body_multineat_params.MutateRemLinkProb = 0.02
    body_multineat_params.RecurrentProb = 0.0
    body_multineat_params.OverallMutationRate = 0.15
    body_multineat_params.MutateAddLinkProb = 0.08
    body_multineat_params.MutateAddNeuronProb = 0.01
    body_multineat_params.MutateWeightsProb = 0.90
    body_multineat_params.MaxWeight = 8.0
    body_multineat_params.WeightMutationMaxPower = 0.2
    body_multineat_params.WeightReplacementMaxPower = 1.0
    body_multineat_params.MutateActivationAProb = 0.0
    body_multineat_params.ActivationAMutationMaxPower = 0.5
    body_multineat_params.MinActivationA = 0.05
    body_multineat_params.MaxActivationA = 6.0

    body_multineat_params.MutateNeuronActivationTypeProb = 0.03

    body_multineat_params.ActivationFunction_SignedSigmoid_Prob = 0.0
    body_multineat_params.ActivationFunction_UnsignedSigmoid_Prob = 0.0
    body_multineat_params.ActivationFunction_Tanh_Prob = 1.0
    body_multineat_params.ActivationFunction_TanhCubic_Prob = 0.0
    body_multineat_params.ActivationFunction_SignedStep_Prob = 1.0
    body_multineat_params.ActivationFunction_UnsignedStep_Prob = 0.0
    body_multineat_params.ActivationFunction_SignedGauss_Prob = 1.0
    body_multineat_params.ActivationFunction_UnsignedGauss_Prob = 0.0
    body_multineat_params.ActivationFunction_Abs_Prob = 0.0
    body_multineat_params.ActivationFunction_SignedSine_Prob = 1.0
    body_multineat_params.ActivationFunction_UnsignedSine_Prob = 0.0
    body_multineat_params.ActivationFunction_Linear_Prob = 1.0

    body_multineat_params.MutateNeuronTraitsProb = 0.0
    body_multineat_params.MutateLinkTraitsProb = 0.0

    body_multineat_params.AllowLoops = False

    # brain multineat settings
    brain_multineat_params = multineat.Parameters()

    brain_multineat_params.MutateRemLinkProb = 0.02
    brain_multineat_params.RecurrentProb = 0.0
    brain_multineat_params.OverallMutationRate = 0.15
    brain_multineat_params.MutateAddLinkProb = 0.08
    brain_multineat_params.MutateAddNeuronProb = 0.01
    brain_multineat_params.MutateWeightsProb = 0.90
    brain_multineat_params.MaxWeight = 8.0
    brain_multineat_params.WeightMutationMaxPower = 0.2
    brain_multineat_params.WeightReplacementMaxPower = 1.0
    brain_multineat_params.MutateActivationAProb = 0.0
    brain_multineat_params.ActivationAMutationMaxPower = 0.5
    brain_multineat_params.MinActivationA = 0.05
    brain_multineat_params.MaxActivationA = 6.0

    brain_multineat_params.MutateNeuronActivationTypeProb = 0.03

    brain_multineat_params.ActivationFunction_SignedSigmoid_Prob = 0.0
    brain_multineat_params.ActivationFunction_UnsignedSigmoid_Prob = 0.0
    brain_multineat_params.ActivationFunction_Tanh_Prob = 1.0
    brain_multineat_params.ActivationFunction_TanhCubic_Prob = 0.0
    brain_multineat_params.ActivationFunction_SignedStep_Prob = 1.0
    brain_multineat_params.ActivationFunction_UnsignedStep_Prob = 0.0
    brain_multineat_params.ActivationFunction_SignedGauss_Prob = 1.0
    brain_multineat_params.ActivationFunction_UnsignedGauss_Prob = 0.0
    brain_multineat_params.ActivationFunction_Abs_Prob = 0.0
    brain_multineat_params.ActivationFunction_SignedSine_Prob = 1.0
    brain_multineat_params.ActivationFunction_UnsignedSine_Prob = 0.0
    brain_multineat_params.ActivationFunction_Linear_Prob = 1.0

    brain_multineat_params.MutateNeuronTraitsProb = 0.0
    brain_multineat_params.MutateLinkTraitsProb = 0.0

    brain_multineat_params.AllowLoops = False

    # bodybrain composition genotype config
    bodybrain_composition_config = BodybrainCompositionConfig(
        body_crossover=multineat_body_crossover,
        brain_crossover=multineat_cpg_brain_crossover,
        body_crossover_config=None,
        brain_crossover_config=None,
        body_mutate=multineat_body_mutate,
        brain_mutate=multineat_cpg_brain_mutate,
        body_mutate_config=None,
        brain_mutate_config=None,
        body_develop=multineat_body_develop,
        brain_develop=multineat_cpg_brain_develop,
        body_develop_config=None,
        brain_develop_config=brain_config,
    )

    # genotype constructor config. Used by `create_random_genotype` in this file.
    genotype_constructor_config = GenotypeConstructorConfig(
        bodybrain_composition_config, body_multineat_params, brain_multineat_params
    )

    ###### From here on I did not check things yet --Aart

    # Parse command line / file input arguments
    settings = parser.parse_args()
    experiment_management = ExperimentManagement(settings)
    do_recovery = (
        settings.recovery_enabled and not experiment_management.experiment_is_new()
    )

    logger.info(
        "Activated run " + settings.run + " of experiment " + settings.experiment_name
    )

    if do_recovery:
        (
            gen_num,
            has_offspring,
            next_robot_id,
        ) = experiment_management.read_recovery_state(population_size, offspring_size)

        if gen_num == num_generations - 1:
            logger.info("Experiment is already complete.")
            return
    else:
        gen_num = 0
        next_robot_id = 1

    population_conf = PopulationConfig(
        population_size=population_size,
        genotype_constructor=create_random_genotype,
        genotype_conf=genotype_constructor_config,
        fitness_function=fitness.displacement_velocity,
        mutation_operator=bodybrain_composition_mutate,
        mutation_conf=bodybrain_composition_config,
        crossover_operator=bodybrain_composition_crossover,
        crossover_conf=bodybrain_composition_config,
        selection=lambda individuals: tournament_selection(individuals, 2),
        parent_selection=lambda individuals: multiple_selection(
            individuals, 2, tournament_selection
        ),
        population_management=steady_state_population_management,
        population_management_selector=tournament_selection,
        evaluation_time=settings.evaluation_time,
        offspring_size=offspring_size,
        experiment_name=settings.experiment_name,
        experiment_management=experiment_management,
    )

    n_cores = settings.n_cores

    simulator_queue = SimulatorQueue(n_cores, settings, settings.port_start)
    await simulator_queue.start()

    analyzer_queue = None  # AnalyzerQueue(1, settings, settings.port_start + n_cores)
    # await analyzer_queue.start()

    population = Population(
        population_conf, simulator_queue, analyzer_queue, next_robot_id
    )

    if do_recovery:
        # loading a previous state of the experiment
        await population.load_snapshot(gen_num)
        if gen_num >= 0:
            logger.info(
                "Recovered snapshot "
                + str(gen_num)
                + ", pop with "
                + str(len(population.individuals))
                + " individuals"
            )
        if has_offspring:
            individuals = await population.load_offspring(
                gen_num, population_size, offspring_size, next_robot_id
            )
            gen_num += 1
            logger.info("Recovered unfinished offspring " + str(gen_num))

            if gen_num == 0:
                await population.init_pop(individuals)
            else:
                population = await population.next_gen(gen_num, individuals)

            experiment_management.export_snapshots(population.individuals, gen_num)
    else:
        # starting a new experiment
        experiment_management.create_exp_folders()
        await population.init_pop()
        experiment_management.export_snapshots(population.individuals, gen_num)

    while gen_num < num_generations - 1:
        gen_num += 1
        population = await population.next_gen(gen_num)
        experiment_management.export_snapshots(population.individuals, gen_num)

    # output result after completing all generations...
