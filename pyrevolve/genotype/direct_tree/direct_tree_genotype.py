from typing import Optional

from pyrevolve.genotype import Genotype
from pyrevolve.genotype.direct_tree.direct_tree_config import DirectTreeGenotypeConfig

from pyrevolve.genotype.direct_tree import direct_tree_random_generator
from pyrevolve.revolve_bot import RevolveBot
from pyrevolve.revolve_bot.revolve_module import ActiveHingeModule, RevolveModule
from pyrevolve.revolve_bot.brain import BrainNN, brain_nn
from pyrevolve.revolve_bot.revolve_module import CoreModule


class DirectTreeGenotype(Genotype):

    def __init__(self, conf: DirectTreeGenotypeConfig, robot_id: Optional[int]):
        """
        :param conf: configurations for l-system
        :param robot_id: unique id of the robot
        :type conf: PlasticodingConfig
        """
        self.conf: DirectTreeGenotypeConfig = conf
        assert robot_id is None or str(robot_id).isdigit()
        self.id: int = int(robot_id) if robot_id is not None else -1

        self.representation: CoreModule = CoreModule()

        # Auxiliary variables
        self.valid: bool = False

        self.phenotype: Optional[RevolveBot] = None

    def clone(self):
        # Cannot use deep clone for this genome, because it is bugged sometimes
        _id = self.id if self.id >= 0 else None

        other = DirectTreeGenotype(self.conf, _id)
        other.valid = self.valid
        other.representation = self.representation

        other.phenotype = self.phenotype
        return other

    def load_genotype(self, genotype_filename: str) -> None:
        revolvebot: RevolveBot = RevolveBot()
        revolvebot.load_file(genotype_filename, conf_type='yaml')
        self.id = revolvebot.id
        self.representation = revolvebot._body

        # load brain params into the modules
        brain = revolvebot._brain
        assert isinstance(brain, BrainNN)

        module_map = {}
        for module in revolvebot.iter_all_elements():
            module_map[module.id] = module

        for node_id, oscillator in brain.params.items():
            node = brain.nodes[node_id]
            module_map[node.part_id].oscillator_amplitude = oscillator.amplitude
            module_map[node.part_id].oscillator_period = oscillator.period
            module_map[node.part_id].oscillator_phase = oscillator.phase_offset

    def export_genotype(self, filepath: str) -> None:
        self.develop()
        self.phenotype.save_file(filepath, conf_type='yaml')

    def develop(self) -> RevolveBot:
        if self.phenotype is None:
            self.phenotype: RevolveBot = RevolveBot(self.id)
            self.phenotype._body: CoreModule = self.representation
            self.phenotype._brain = self._develop_brain(self.representation)
        return self.phenotype

    def _develop_brain(self, core: CoreModule):
        brain = BrainNN()

        for module in self.phenotype.iter_all_elements():
            if isinstance(module, ActiveHingeModule):
                node = brain_nn.Node()
                node.id = f'node_{module.id}'
                node.part_id = module.id

                node.layer = 'output'
                node.type = 'Oscillator'

                params = brain_nn.Params()
                params.period = module.oscillator_period
                params.phase_offset = module.oscillator_phase
                params.amplitude = module.oscillator_amplitude
                node.params = params

                brain.params[node.id] = params
                brain.nodes[node.id] = node

        # add imu sensor stuff or the brain fail to load
        for p in range(1, 7):
            _id = 'IMU_' + str(p)
            node = brain_nn.Node()
            node.layer = 'input'
            node.type = 'Input'
            node.part_id = core.id
            node.id = _id
            brain.nodes[_id] = node

        return brain

    def random_initialization(self):
        self.representation = direct_tree_random_generator.generate_tree(self.representation, config=self.conf)
        return self