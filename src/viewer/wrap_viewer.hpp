#pragma once

/*
 * Copyright (c) 2025, LaneZero Contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pybind11/pybind11.h>

#include <QPointer>
#include <QMenu>

#ifndef LANEZERO_PYSIDE6_FULL
namespace PySide
{
PyTypeObject * getTypeForQObject(const QObject * cppSelf);
PyObject * getWrapperForQObject(QObject * cppSelf, PyTypeObject * sbk_type);
QObject * convertToQObject(PyObject * object, bool raiseError);
} /* end namespace PySide */
#endif /* LANEZERO_PYSIDE6_FULL */

namespace pybind11
{

namespace detail
{

template <typename type>
struct qt_type_caster
{
protected:
    type * value;

public:
    template <typename T_, enable_if_t<std::is_same<type, remove_cv_t<T_>>::value, int> = 0>
    static handle cast(T_ * src, return_value_policy policy, handle parent)
    {
        if (!src)
            return none().release();
        if (policy == return_value_policy::take_ownership)
        {
            auto h = cast(std::move(*src), policy, parent);
            delete src;
            return h;
        }
        else
        {
            return cast(*src, policy, parent);
        }
    }
    
    template <typename T_>
    using cast_op_type = pybind11::detail::movable_cast_op_type<T_>;

    bool load(handle src, bool)
    {
        if (!src)
        {
            return false;
        }

        QObject * q = PySide::convertToQObject(src.ptr(), /* raiseError */ true);
        if (!q)
        {
            return false;
        }

        value = qobject_cast<type *>(q);
        return true;
    }

    static handle cast(type const & src, return_value_policy /* policy */, handle /* parent */)
    {
        return cast(&src);
    }

    static handle cast(type const * src, return_value_policy /* policy */ = return_value_policy::automatic_reference, handle /* parent */ = handle())
    {
        if (!src)
        {
            return none().inc_ref();
        }

        auto * cppSelf = const_cast<type *>(src);
        PyTypeObject * sbk_type = PySide::getTypeForQObject(cppSelf);
        PyObject * pyOut = PySide::getWrapperForQObject(cppSelf, sbk_type);
        if (!pyOut)
        {
            throw std::runtime_error("Failed to create Python wrapper for QObject");
        }

        return handle(pyOut);
    }

    static constexpr auto name = const_name("PySide6.QtCore.QObject");
};

} /* end namespace detail */

} /* end namespace pybind11 */

#define LANEZERO_DECL_QT_TYPE_CASTER(QT_TYPE) \
    namespace pybind11 \
    { \
    namespace detail \
    { \
    template <> \
    struct type_caster<QT_TYPE> : public qt_type_caster<QT_TYPE> \
    { \
    }; \
    template <> \
    struct type_caster<QT_TYPE *> : public qt_type_caster<QT_TYPE> \
    { \
    }; \
    } /* end namespace detail */ \
    } /* end namespace pybind11 */

#include <QMainWindow>
#include <QMenu>
#include <QCoreApplication>

LANEZERO_DECL_QT_TYPE_CASTER(QMainWindow)
LANEZERO_DECL_QT_TYPE_CASTER(QMenu)
LANEZERO_DECL_QT_TYPE_CASTER(QCoreApplication)

// vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
