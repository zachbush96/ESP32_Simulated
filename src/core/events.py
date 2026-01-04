from dataclasses import dataclass
from typing import Any, Callable, DefaultDict, Dict, List


@dataclass
class Event:
    name: str
    payload: Dict[str, Any]


class EventBus:
    def __init__(self) -> None:
        self._subscribers: DefaultDict[str, List[Callable[[Event], None]]] = DefaultDict(list)

    def subscribe(self, name: str, handler: Callable[[Event], None]) -> None:
        self._subscribers[name].append(handler)

    def publish(self, event: Event) -> None:
        for handler in self._subscribers[event.name]:
            handler(event)
