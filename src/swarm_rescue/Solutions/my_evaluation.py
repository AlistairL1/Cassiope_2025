from typing import Tuple
import gc
from spg_overlay.utils.constants import DRONE_INITIAL_HEALTH
from spg_overlay.reporting.evaluation import EvalConfig
from spg_overlay.reporting.score_manager import ScoreManager
from spg_overlay.gui_map.gui_sr import GuiSR

from maps.map_intermediate_01 import MyMapIntermediate01
from maps.map_intermediate_02 import MyMapIntermediate02
from maps.my_custom_map import CustomMap02
from maps.map_gotta_hurry_01 import MapNarrowCorridor
from maps.map_gotta_hurry_02 import MapMaze

from solutions.my_drone_eval import MyDroneEval


def evaluate_drone(nb_rounds: int = 2, nb_drones: int = 5, map_type=MyMapIntermediate02, nb_wounded: int = 20):
    """
    Évalue les performances des drones sur la carte spécifiée

    Args:
        nb_rounds: Nombre de rounds à exécuter
        nb_drones: Nombre de drones à utiliser
        map_type: Type de carte à utiliser
        nb_wounded: Nombre de blessés à placer
    """
    # Configuration de l'évaluation
    zones_config: Tuple = ()  # Pas de zones spéciales
    eval_config = EvalConfig(map_type=map_type, zones_config=zones_config, nb_rounds=nb_rounds)

    # Résultats globaux
    all_scores = []
    all_rescued = []
    all_exploration = []
    all_health = []
    all_timesteps = []

    print("\nDébut de l'évaluation...")
    print(f"Carte: {map_type.__name__}")
    print(f"Nombre de rounds: {nb_rounds}")
    print(f"Nombre de drones: {nb_drones}")
    print(f"Nombre de blessés: {nb_wounded}")

    for num_round in range(nb_rounds):
        print(f"\nRound {num_round + 1}/{nb_rounds}")

        # Création de la carte avec le nombre de drones spécifié
        my_map = eval_config.map_type(eval_config.zones_config)
        
        # Pour MyMapIntermediate02, on peut ajuster le nombre de blessés
        # mais pas le nombre de drones directement
        if map_type == MyMapIntermediate02:
            # Modifier l'attribut interne pour MyMapIntermediate02
            my_map._number_drones = nb_drones
            
            # Limiter le nombre de blessés à la taille de la liste disponible
            max_available = len(my_map._wounded_persons_pos)
            actual_wounded = min(nb_wounded, max_available)
            my_map._number_wounded_persons = actual_wounded
            my_map._wounded_persons_pos = my_map._wounded_persons_pos[:actual_wounded]
            print(f"Nombre réel de blessés: {actual_wounded}")
        elif map_type not in [MapNarrowCorridor, MapMaze, CustomMap02] :  # Ne pas modifier CustomMap02 qui est déjà configurée
            # Pour les autres types de carte comme MyMapIntermediate01
            my_map.number_drones = nb_drones

        # Configuration du score manager
        score_manager = ScoreManager(
            number_drones=my_map.number_drones,
            max_timestep_limit=my_map.max_timestep_limit,
            max_walltime_limit=my_map.max_walltime_limit,
            total_number_wounded_persons=my_map.number_wounded_persons
        )

        # Création du playground
        my_playground = my_map.construct_playground(drone_type=MyDroneEval)

        # Création de l'interface
        my_gui = GuiSR(
            playground=my_playground,
            the_map=my_map,
            use_keyboard=False,
            draw_interactive=True,  # Activé pour voir les drones en action
            enable_visu_noises=False,
        )

        # Reset de la carte explorée
        my_map.explored_map.reset()

        # Exécution de la simulation
        my_gui.run()

        # Calcul des scores
        score_exploration = my_map.explored_map.score() * 100.0
        score_health_returned = my_map.compute_score_health_returned() * 100
        mean_drones_health_percent = my_gui.mean_drones_health / DRONE_INITIAL_HEALTH * 100.

        # Calcul du score final
        result_score = score_manager.compute_score(
            my_gui.rescued_number,
            score_exploration,
            score_health_returned,
            my_gui.full_rescue_timestep
        )
        round_score, percent_rescued, score_timestep = result_score

        # Sauvegarde des résultats
        all_scores.append(round_score)
        all_rescued.append(percent_rescued)
        all_exploration.append(score_exploration)
        all_health.append(mean_drones_health_percent)
        all_timesteps.append(my_gui.elapsed_timestep)

        # Affichage des résultats du round
        print(f"\nRésultats du round {num_round + 1}:")
        print(f"  Score final: {round_score:.1f}%")
        print(f"  Personnes sauvées: {my_gui.rescued_number}/{my_map.number_wounded_persons} ({percent_rescued:.1f}%)")
        print(f"  Score d'exploration: {score_exploration:.1f}%")
        print(f"  Santé moyenne des drones: {mean_drones_health_percent:.1f}%")
        print(f"  Pas de temps utilisés: {my_gui.elapsed_timestep}/{my_map.max_timestep_limit}")

        # Nettoyage
        my_gui.close()
        gc.collect()

    # Calcul et affichage des moyennes
    print("\nRésultats moyens sur tous les rounds:")
    print(f"  Score moyen: {sum(all_scores) / len(all_scores):.1f}%")
    print(f"  Pourcentage moyen de sauvetage: {sum(all_rescued) / len(all_rescued):.1f}%")
    print(f"  Score moyen d'exploration: {sum(all_exploration) / len(all_exploration):.1f}%")
    print(f"  Santé moyenne des drones: {sum(all_health) / len(all_health):.1f}%")
    print(f"  Nombre moyen de pas: {sum(all_timesteps) / len(all_timesteps):.0f}")


def evaluate_map01(nb_rounds: int = 2, nb_drones: int = 2):
    """Évalue les performances sur MyMapIntermediate01"""
    evaluate_drone(nb_rounds=nb_rounds, nb_drones=nb_drones, map_type=MyMapIntermediate01)


def evaluate_map02(nb_rounds: int = 2, nb_drones: int = 5, nb_wounded: int = 20):
    """Évalue les performances sur MyMapIntermediate02"""
    evaluate_drone(nb_rounds=nb_rounds, nb_drones=nb_drones, map_type=MyMapIntermediate02, nb_wounded=nb_wounded)


def evaluate_custom_map(nb_rounds: int = 2, map=MyMapIntermediate01):
    """Évalue les performances sur la carte personnalisée avec 2 drones et 2 blessés"""
    evaluate_drone(nb_rounds=nb_rounds, map_type=map, nb_drones=1, nb_wounded=1)


if __name__ == "__main__":
    evaluate_custom_map(nb_rounds=2, map=CustomMap02)
    evaluate_custom_map(nb_rounds=2, map=MapNarrowCorridor)
