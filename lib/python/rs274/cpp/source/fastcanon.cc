#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "structmember.h"

typedef struct {
    PyObject_HEAD
    PyObject *first; /* first name */
    PyObject *last;  /* last name */
    int number;
} FastcanonObject;

static int
Fastcanon_traverse(FastcanonObject *self, visitproc visit, void *arg)
{
    Py_VISIT(self->first);
    Py_VISIT(self->last);
    return 0;
}

static int
Fastcanon_clear(FastcanonObject *self)
{
    Py_CLEAR(self->first);
    Py_CLEAR(self->last);
    return 0;
}

static void
Fastcanon_dealloc(FastcanonObject *self)
{
    PyObject_GC_UnTrack(self);
    Fastcanon_clear(self);
    Py_TYPE(self)->tp_free((PyObject *) self);
}

static PyObject *
Fastcanon_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    FastcanonObject *self;
    self = (FastcanonObject *) type->tp_alloc(type, 0);
    if (self != NULL) {
        self->first = PyUnicode_FromString("");
        if (self->first == NULL) {
            Py_DECREF(self);
            return NULL;
        }
        self->last = PyUnicode_FromString("");
        if (self->last == NULL) {
            Py_DECREF(self);
            return NULL;
        }
        self->number = 0;
    }
    return (PyObject *) self;
}

static int
Fastcanon_init(FastcanonObject *self, PyObject *args, PyObject *kwds)
{
    static const char *kwlist[] = {"first", "last", "number", NULL};
    PyObject *first = NULL, *last = NULL, *tmp;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|UUi", (char**)kwlist,
                                     &first, &last,
                                     &self->number))
        return -1;

    if (first) {
        tmp = self->first;
        Py_INCREF(first);
        self->first = first;
        Py_DECREF(tmp);
    }
    if (last) {
        tmp = self->last;
        Py_INCREF(last);
        self->last = last;
        Py_DECREF(tmp);
    }
    return 0;
}

static PyMemberDef Fastcanon_members[] = {
        {"number", T_INT, offsetof(FastcanonObject, number), 0,
                "fastcanon number"},
        {NULL}  /* Sentinel */
};

static PyObject *
Fastcanon_getfirst(FastcanonObject *self, void *closure)
{
    Py_INCREF(self->first);
    return self->first;
}

static int
Fastcanon_setfirst(FastcanonObject *self, PyObject *value, void *closure)
{
    if (value == NULL) {
        PyErr_SetString(PyExc_TypeError, "Cannot delete the first attribute");
        return -1;
    }
    if (!PyUnicode_Check(value)) {
        PyErr_SetString(PyExc_TypeError,
                        "The first attribute value must be a string");
        return -1;
    }
    Py_INCREF(value);
    Py_CLEAR(self->first);
    self->first = value;
    return 0;
}

static PyObject *
Fastcanon_getlast(FastcanonObject *self, void *closure)
{
    Py_INCREF(self->last);
    return self->last;
}

static int
Fastcanon_setlast(FastcanonObject *self, PyObject *value, void *closure)
{
    if (value == NULL) {
        PyErr_SetString(PyExc_TypeError, "Cannot delete the last attribute");
        return -1;
    }
    if (!PyUnicode_Check(value)) {
        PyErr_SetString(PyExc_TypeError,
                        "The last attribute value must be a string");
        return -1;
    }
    Py_INCREF(value);
    Py_CLEAR(self->last);
    self->last = value;
    return 0;
}

static PyGetSetDef Fastcanon_getsetters[] = {
        {"first", (getter) Fastcanon_getfirst, (setter) Fastcanon_setfirst,
                "first name", NULL},
        {"last", (getter) Fastcanon_getlast, (setter) Fastcanon_setlast,
                "last name", NULL},
        {NULL}  /* Sentinel */
};

static PyObject *
Fastcanon_name(FastcanonObject *self, PyObject *Py_UNUSED(ignored))
{
    return PyUnicode_FromFormat("%S %S", self->first, self->last);
}

static PyMethodDef Fastcanon_methods[] = {
        {"name", (PyCFunction) Fastcanon_name, METH_NOARGS,
                "Return the name, combining the first and last name"
        },
        {NULL}  /* Sentinel */
};

static PyTypeObject FastcanonType = {
        PyVarObject_HEAD_INIT(NULL, 0)

        .tp_name = "fastcanon.Fastcanon",
        .tp_basicsize = sizeof(FastcanonObject),
        .tp_itemsize = 0,
        .tp_dealloc = (destructor) Fastcanon_dealloc,

        .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HAVE_GC,
        .tp_doc = "Fastcanon objects",
        .tp_traverse = (traverseproc) Fastcanon_traverse,
        .tp_clear = (inquiry) Fastcanon_clear,
        .tp_methods = Fastcanon_methods,
        .tp_members = Fastcanon_members,
        .tp_getset = Fastcanon_getsetters,
        .tp_init = (initproc) Fastcanon_init,
        .tp_new = Fastcanon_new,
};

static PyModuleDef fastcanonmodule = {
        PyModuleDef_HEAD_INIT,
        .m_name = "fastcanon",
        .m_doc = "Fast Canon Implementation for LinuxCNC visualization",
        .m_size = -1,
};

PyMODINIT_FUNC
PyInit_fastcanon(void)
{
    PyObject *m;
    if (PyType_Ready(&FastcanonType) < 0)
        return NULL;

    m = PyModule_Create(&fastcanonmodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&FastcanonType);
    PyModule_AddObject(m, "Fastcanon", (PyObject *) &FastcanonType);
    return m;
}