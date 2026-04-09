"""Interactive panel to list and run examples from the `examples` folder.

Expected Structure:
- Each file in `examples/` should expose an `app()` function that instantiates and runs the test.

Usage:
- Run this file with `python main.py` and select an example by number.
"""
import importlib
import sys
from pathlib import Path
from coppelia_sim_framework import setup_logger

logger = setup_logger(__name__, '[MAIN]')


def list_examples():
    """Reads the 'examples' folder and returns a list of example filenames."""
    examples_folder = Path("examples")
    examples = []
    
    # Check if the folder exists before trying to read it
    if not examples_folder.exists() or not examples_folder.is_dir():
        logger.warning("Warning: The 'examples' folder was not found.")
        return examples
        
    # Search for all .py files in the examples folder
    for file in examples_folder.glob("*.py"):
        file_name = file.stem  # Gets only the name (ignores .py)
        
        # Ignores __init__.py and hidden files
        if file_name != "__init__" and not file_name.startswith("."):
            examples.append(file_name)
            
    # Returns in alphabetical order for an organized menu
    return sorted(examples)


def main():
    """Main interactive loop.

    - Lists modules in `examples/` (Python files, except __init__).
    - Presents menu and dynamically imports the chosen module.
    - Looks for an `app()` function in the module to run the example.
    """

    # 1. Dynamically reads the folder
    examples_list = list_examples()
    
    if not examples_list:
        logger.error("No examples found in the 'examples/' folder.")
        return

    # 2. Builds the Menu
    print("\n" + "="*50)
    print(" COPPELIASIM FRAMEWORK - EXAMPLES MENU")
    print("="*50)
    print("Select the example you want to run:\n")
    
    for i, example_name in enumerate(examples_list):
        print(f"[{i + 1}] - {example_name}")
        
    print("[0] - Exit")
    print("-" * 50)
    
    # 3. Interaction loop
    while True:
        choice = input("Enter the option number: ")
        
        if choice == '0':
            logger.info("Exiting the panel. See you later!")
            break
            
        if choice.isdigit():
            index = int(choice) - 1
            
            if 0 <= index < len(examples_list):
                selected_name = examples_list[index]
                logger.info(f"Starting example: {selected_name}")  
                
                # 4. Dynamic Import and Execution
                try:
                    # Imports the chosen module (Ex: from examples import locomocao_example)
                    module = importlib.import_module(f"examples.{selected_name}")
                    
                    # Checks if you really created the 'app()' function inside the file
                    if hasattr(module, 'app'):
                        module.app()
                    else:
                        logger.error(f"File '{selected_name}.py' does not have an 'app()' function")
                        
                except ImportError as e:
                    logger.error(f"Error loading example '{selected_name}': {e}")
                except Exception as e:
                    logger.error(f"Critical error running example '{selected_name}': {type(e).__name__}: {e}")
                    
                break  # Exits the menu after running the example
            else:
                logger.error("Number out of range. Try again.")
        else:
            logger.error("Please enter only valid numbers.")


if __name__ == '__main__':
    main()