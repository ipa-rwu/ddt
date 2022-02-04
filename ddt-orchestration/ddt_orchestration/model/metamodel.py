from __future__ import annotations
import itertools

def _get_class_name(c):
    return c if isinstance(c, str) else c.__name__

def has_class(inst, classnames):
    if not isinstance(classnames, (tuple, list)):
        classnames=[classnames]
    classnames =  set(map(_get_class_name, classnames))
    for c in inst.__class__.mro():
        if _get_class_name(c) in classnames:
            return True
    return False

def has_type(inst, typenames):
    if isinstance(typenames, str):
        typenames=[typenames]
    for b in inst.iter_types():
        if b.name in typenames:
            return True
    return False

def has_type_or_class(inst, names):
    return has_type(inst, names) or has_class(inst, names)

class _FixedDict(dict):
    def __setitem__(self, key, value):
        raise RuntimeError(f"Cannot update keys")
    def __delitem__(self, key):
        raise RuntimeError(f"Cannot remove keys")
    def dict(self):
        return dict((k,v.dict()) for k,v in self.items())
    def __getattr__(self, name):
        return self.get(name)

class _Dict(dict):
    def __getattr__(self, name):
        return self[name]
    def dict(self):
        return [ v.dict() for v in self.values() ]
        #return dict( (k, v.dict()) for k, v in self.items())
    def extend(self, iterable):
        for e in iterable:
            self.add(e)

class _ElementDict (_Dict):
    def __init__(self, type, scope, iterable=[]):
        super().__init__()
        self.scope = scope
        self.type = type
        self.extend(iterable)
    def add(self, element):
        if not has_class(element.type, self.type):
            raise TypeError(f"element type is not '{self.type}'")
        self[element.name] = self._inject_scope(element)
    def _inject_scope(self, elem):
        elem.scope = self.scope + elem.scope
        return elem

class _EntityDict (_Dict):
    def __init__(self, type, parent, name, iterable=[]):
        super().__init__()
        self.type = type
        self.parent = parent
        self.name = name
        setattr(parent, name, self)
        self.extend(iterable)
    def add(self, element):
        if not has_class(element.type, self.type):
            raise TypeError(f"element type is not '{self.type}'")
        instance = element.type.instantiate(element, self.parent)
        instance.scope = [self.name]
        self[element.name] = instance
        return instance
    def filter(self, filter):
        for v in self.values():
            if has_type(v, filter) or has_class(v, filter):
                yield v

class _ScopeElement:
    def __init__(self, name: str, scope: List[str] = []):
        self.name = name
        self.scope = scope
    def get_path(self):
        return (*self.scope, self.name)
    def fqn(self, sep='.'):
        return sep.join(self.get_path())
    def __hash__(self):
        return hash(self.fqn())
    def __repr__(self):
        return f"{self.__class__.__name__}('{self.fqn()}')"

class _HierarchyElement:
    def __init__(self, name: str, parent=None, scope=[]):
        self.name = name
        self.parent = parent
        self.scope = scope
    def _iter_parents(self, relative_to=None):
        parent = self.parent
        while parent:
            if parent == relative_to:
                break
            yield parent
            parent = parent.parent
        else:
            if relative_to is not None:
                raise RuntimeError(f"{parent} not found")
    def _get_parents(self, relative_to, scopes):
        for p in reversed(list(self._iter_parents(relative_to))):
            if scopes:
                yield from p.scope
            yield p.name
    def get_path(self, relative_to=None, scopes=False):
        return (*self._get_parents(relative_to, scopes), *(self.scope if scopes else []), self.name)
    def fqn(self, relative_to=None, sep='.', scopes=False):
        return sep.join(self.get_path(relative_to, scopes))
    def __hash__(self):
        return hash(self.fqn())
    def __repr__(self):
        return f"{self.__class__.__name__}('{self.fqn()}')"
    def __lt__(self, other):
        return self.fqn() < other.fqn()
    def get_parent_of_type(self, type):
        for parent in [*self._iter_parents()]:
            if has_type_or_class(parent, type):
                return parent
        raise RuntimeError(f"Could not find parent of {self} with type(s) '{type}'")

class Element(_ScopeElement):
    def __init__(self, name: str, type, **kwargs):
        super().__init__(name)
        self.type = type
        self.kwargs = kwargs
    def dict(self):
        return dict(name=self.name, type=self.type, **self.kwargs)

def _collect_refs(type):
    for c in reversed(type.__class__.mro()):
        for k, v in getattr(c, "REFS", {}).items():
            yield k, (k, v)

def _collect_props(type):
    for c in reversed(type.__class__.mro()):
        for k, v in getattr(c, "PROPS", {}).items():
            yield k, (k, v)

def _collect_fields(type):
    for c in reversed(type.__class__.mro()):
        yield from getattr(c, 'FIELDS', [])

class _EntityType(_ScopeElement):
    REFS = {}
    PROPS = {}
    FIELDS = []
    def __init__(self, name: str, base: _EntityType = None, scope: List[str] = [], **kwargs):
        super().__init__(name, scope)
        self._refs = dict(_collect_refs(self))
        self._props = dict(_collect_props(self))
        self._args = list((t,n) for n, t in self.get_refs())
        self._fields = list({f:f for f in _collect_fields(self)})
        self.base = base
        for r, t in self.get_refs():
            setattr(self, r, _ElementDict(t, scope + [name, r], kwargs.get(r,[])))
        for p, t in self.get_props():
            setattr(self, p, _ElementDict(t, scope + [name, p], kwargs.get(p,[])))
        for f in self.get_fields():
            setattr(self, f, kwargs.get(f,None))
        self._postinit()
    def _postinit(self):
        pass
    def get_refs(self):
        return self._refs.values()
    def get_props(self):
        return self._props.values()
    def get_fields(self):
        return self._fields
    def iter_types(self, to_base=None):
        base = self
        while base:
            yield base
            if to_base == base:
                break
            base = base.base
        else:
            if to_base is not None:
                raise RuntimeError(f"{to_base} not found")
    def has_base(self, base):
        try:
            for b in self.iter_types(base):
                pass
            return True
        except RuntimeError:
            return False
    def dict(self):
        def _make_dict():
            yield 'name', self.name
            yield 'scope', self.scope
            for r, __ in self.get_refs():
                yield r, getattr(self, r).dict()
            for p, __ in self.get_props():
                yield p, getattr(self, p).dict()
            for f in self.get_fields():
                yield f, getattr(self, f)
        return dict(_make_dict())
    def derive(self, name, scope=None, **kwargs):
        return self.__class__(name=name, scope=scope if scope is not None else self.scope, base=self, **kwargs)
    def instantiate(self, element, parent):
        return self.ENTITY(element.name, element.type, parent, **element.kwargs)
    def get_class_name(self):
        return self.ENTITY.__name__

    def build_args(self, elements):
        args = dict((n, []) for _,n in self._args)
        for e in elements:
            for t, n in self._args:
                if has_class(e.type, t):
                    args[n].append(e)
                    break
            else:
                raise RuntimeError(f"Cannot add '{e.name}' of type '{e.type}' to {self}")
        return args

class _Entity(_HierarchyElement):
    def __init__(self, name, type, parent, **kwargs):
        super().__init__(name, parent)
        self.type = type
        for n, t in itertools.chain(type.get_refs(), type.get_props()):
            def _gen():
                for base in reversed(list(type.iter_types())):
                    yield from getattr(base, n, dict()).values()
                yield from kwargs.get(n, [])
            _EntityDict(t, self, n, _gen())
        self._postinit()
    def _postinit(self):
        pass
    def iter_types(self, to_base=None):
        yield from self.type.iter_types(to_base)
    def dict(self):
        def _make_dict():
            yield 'name', self.name
            yield 'parent', self.parent
            yield 'type', self.type
            for r, __ in self.type.get_refs():
                yield r, getattr(self, r).dict()
            for p, __ in self.type.get_props():
                yield p, getattr(self, p).dict()
        return dict(_make_dict())

    def walk(self, filter_results=None, exclude_walk=None):
        for n, t in self.type.get_refs():
            for e in getattr(self, n).values():
                if isinstance(e, _Entity):
                    if filter_results is None or has_type_or_class(e, filter_results):
                        yield e
                    if exclude_walk is None or not has_type_or_class(e, exclude_walk):
                        yield from e.walk(filter_results, exclude_walk)

    def find_direct(self, p):
        for n, t in self.type.get_refs():
            for e in getattr(self, n).values():
                if e.name == p:
                    return e
        raise KeyError(p)

    def find(self, path0, *paths):
        elem = self
        for p in [path0, *paths]:
            elem = elem.find_direct(p)
        return elem
