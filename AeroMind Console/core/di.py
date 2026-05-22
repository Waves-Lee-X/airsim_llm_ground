from __future__ import annotations

from typing import Any, Callable, Type, TypeVar

T = TypeVar("T")


class DependencyContainer:
    def __init__(self) -> None:
        self._singletons: dict[Type, Any] = {}
        self._factories: dict[Type, Callable[[], Any]] = {}

    def register_singleton(self, cls: Type[T], instance: T) -> None:
        self._singletons[cls] = instance

    def register_factory(self, cls: Type[T], factory: Callable[[], T]) -> None:
        self._factories[cls] = factory

    def resolve(self, cls: Type[T]) -> T:
        if cls in self._singletons:
            return self._singletons[cls]
        if cls in self._factories:
            instance = self._factories[cls]()
            self._singletons[cls] = instance
            return instance
        raise RuntimeError(f"No registration found for {cls.__name__}")

    def try_resolve(self, cls: Type[T]) -> T | None:
        try:
            return self.resolve(cls)
        except RuntimeError:
            return None


class ServiceManager:
    def __init__(self) -> None:
        self._container = DependencyContainer()
        self._initialized = False

    def register(self, cls: Type[T], instance: T) -> None:
        self._container.register_singleton(cls, instance)

    def register_lazy(self, cls: Type[T], factory: Callable[[], T]) -> None:
        self._container.register_factory(cls, factory)

    def get(self, cls: Type[T]) -> T:
        return self._container.resolve(cls)

    def has(self, cls: Type[T]) -> bool:
        return cls in self._container._singletons or cls in self._container._factories

    def initialize(self) -> None:
        if self._initialized:
            return
        for cls in list(self._container._factories.keys()):
            self._container.resolve(cls)
        self._initialized = True


class Inject:
    def __init__(self, cls: Type[T]) -> None:
        self._cls = cls

    def __get__(self, obj: Any, objtype: Type[Any] | None = None) -> Any:
        if obj is None:
            return self
        if not hasattr(obj, "_service_manager"):
            raise RuntimeError("Object must have _service_manager attribute")
        return obj._service_manager.get(self._cls)