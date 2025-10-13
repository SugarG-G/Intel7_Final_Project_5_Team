// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "turtlebot3_applications_msgs/srv/detail/take_panorama__struct.h"
#include "turtlebot3_applications_msgs/srv/detail/take_panorama__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool turtlebot3_applications_msgs__srv__take_panorama__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[69];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("turtlebot3_applications_msgs.srv._take_panorama.TakePanorama_Request", full_classname_dest, 68) == 0);
  }
  turtlebot3_applications_msgs__srv__TakePanorama_Request * ros_message = _ros_message;
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pano_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "pano_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pano_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // snap_interval
    PyObject * field = PyObject_GetAttrString(_pymsg, "snap_interval");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->snap_interval = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rot_vel
    PyObject * field = PyObject_GetAttrString(_pymsg, "rot_vel");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rot_vel = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * turtlebot3_applications_msgs__srv__take_panorama__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TakePanorama_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("turtlebot3_applications_msgs.srv._take_panorama");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TakePanorama_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  turtlebot3_applications_msgs__srv__TakePanorama_Request * ros_message = (turtlebot3_applications_msgs__srv__TakePanorama_Request *)raw_ros_message;
  {  // mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pano_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pano_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pano_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // snap_interval
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->snap_interval);
    {
      int rc = PyObject_SetAttrString(_pymessage, "snap_interval", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rot_vel
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rot_vel);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rot_vel", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/take_panorama__struct.h"
// already included above
// #include "turtlebot3_applications_msgs/srv/detail/take_panorama__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool turtlebot3_applications_msgs__srv__take_panorama__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[70];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("turtlebot3_applications_msgs.srv._take_panorama.TakePanorama_Response", full_classname_dest, 69) == 0);
  }
  turtlebot3_applications_msgs__srv__TakePanorama_Response * ros_message = _ros_message;
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * turtlebot3_applications_msgs__srv__take_panorama__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TakePanorama_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("turtlebot3_applications_msgs.srv._take_panorama");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TakePanorama_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  turtlebot3_applications_msgs__srv__TakePanorama_Response * ros_message = (turtlebot3_applications_msgs__srv__TakePanorama_Response *)raw_ros_message;
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
