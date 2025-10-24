"""
Key types for the estimator.
"""
from attrs import define, field, validators
from ..utils.validation import _check_valid_key


@define
class Key:
    """
    A key uniquely identifying variables in the estimator.
    Keys are strings starting with a capital letter followed by numbers (e.g., "A0", "L1").
    """
    key: str = field(
        validator=validators.and_(
            validators.instance_of(str),
            _check_valid_key,
        ),
        metadata={"description": "The unique key identifier."},
    )

    def __hash__(self) -> int:
        return hash(self.key)

    def __str__(self) -> str:
        return self.key

    def __repr__(self) -> str:
        return f"Key({self.key})"

    def check(self) -> None:
        _check_valid_key(self, "key", self.key)

    @property
    def is_landmark(self) -> bool:
        """Returns True if this key represents a landmark (starts with 'L')."""
        return self.key[0] == "L"

    @property
    def char(self) -> str:
        """Returns the character prefix of the key."""
        return self.key[0]

    @property
    def index(self) -> int:
        """Returns the numeric index of the key."""
        return int(self.key[1:])


@define
class KeyPair:
    """
    A class to represent a pair of keys.
    Used to identify two variables (typically connected by a measurement).
    """

    key1: Key = field(metadata={"description": "The first key"}, validator=validators.instance_of(Key))
    key2: Key = field(metadata={"description": "The second key"}, validator=validators.instance_of(Key))

    def __str__(self) -> str:
        return f"KeyPair({self.key1}, {self.key2})"
