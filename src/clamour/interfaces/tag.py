"""
Provides a tag interface reference for Clamour
This allows the code to be hardware agnostic. 
All tag classes should follow this blueprint to work with the rest of the codebase.
"""
from abc import ABC, abstractmethod
class Tag(ABC):
    @abstractmethod
    def __init__(self):
        pass 

    @property
    @abstractmethod
    def tag_id(self) -> int:
        """
        Return the tag's unique identifier as an integer.
        """
        pass

    ### -------------------------------------------- DEVICE MANAGEMENT --------------------------------------------

    ### -------------------------------------------- COMMUNICATION --------------------------------------------

    ### -------------------------------------------- LOCALIZATION --------------------------------------------