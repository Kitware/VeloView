#ifndef ENUMMACROS_H
#define ENUMMACROS_H

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/preprocessor.hpp>

//------------------------------------------------------------------------------
// Enum macros.
//------------------------------------------------------------------------------
/*
 * Define some macros to facilitate maintenance of different enum types. These
 * allow a single macro to define an enum type with values, an overloaded
 * ToString function to convert macro values to strings, and a templated
 * To<enum> function to convert integral values to enums.
 *
 * For details of the Boost preprocessing macros, see
 * https://www.boost.org/doc/libs/1_67_0/libs/preprocessor/doc/AppendixA-AnIntroductiontoPreprocessorMetaprogramming.html
 */

//! @brief Internal macro for defining enum values.
#define DEFINE_ENUM_VALUES_INTERNAL(r, prefix, pair) \
  BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)) = BOOST_PP_TUPLE_ELEM(1, pair)

//! @brief Macro for defining enum values.
#define DEFINE_ENUM_VALUES(name, prefix, enumerators) \
  enum name {                                         \
    BOOST_PP_SEQ_ENUM(                                \
      BOOST_PP_SEQ_TRANSFORM(                         \
        DEFINE_ENUM_VALUES_INTERNAL,                  \
        prefix,                                       \
        enumerators                                   \
      )                                               \
    )                                                 \
  };


//! @brief Internal macro for defining enum string conversions.
#define DEFINE_ENUM_STRING_CASES_INTERNAL(r, prefix, pair) \
  case BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)): return BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for defining enum string conversions.
#define DEFINE_ENUM_TO_STRING(name, prefix, enumerators)                                \
  inline                                                                                \
  char const * toString(name const x)                                                   \
  {                                                                                     \
    switch(x)                                                                           \
    {                                                                                   \
      BOOST_PP_SEQ_FOR_EACH(                                                            \
        DEFINE_ENUM_STRING_CASES_INTERNAL,                                              \
        prefix,                                                                         \
        enumerators                                                                     \
      )                                                                                 \
      default: return "<unrecognized enum value of type " BOOST_PP_STRINGIZE(name) ">"; \
    }                                                                                   \
  }


//! @brief Internal macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM_CASES_INTERNAL(r, prefix, pair) \
  case static_cast<T>(BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair))): return BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value) \
  template <typename T>                                                \
  inline                                                               \
  name to ## name(T const x)                                           \
  {                                                                    \
    switch(x)                                                          \
    {                                                                  \
      BOOST_PP_SEQ_FOR_EACH(                                           \
        DEFINE_VALUE_TO_ENUM_CASES_INTERNAL,                           \
        prefix,                                                        \
        enumerators                                                    \
      )                                                                \
      default: return BOOST_PP_CAT(prefix, default_value);             \
    }                                                                  \
  }

/*!
 * @brief Define enum type with string and value conversion functions.
 * @param name        The typename of the enum. This will also be concatenated
 *                    with "To" to define a function that converts integral
 *                    values to this enum type. For example. the name "Foo" will
 *                    define "Foo toFoo(T x)".
 * @param prefix      The prefix to attach to all values of the enum. This may
 *                    be an empty string.
 * @param enumerators A sequence of name-value pairs in double-parentheses, e.g.
 *                    ((a, 1))((b, 2))((c , 4)). All valid enum values may be
 *                    used, e.g. ((a, (1<<0)))((b, (1<<1))), etc.
 */
#define DEFINE_ENUM(name, prefix, enumerators, default_value)    \
  DEFINE_ENUM_VALUES(name, prefix, enumerators)                  \
  DEFINE_ENUM_TO_STRING(name, prefix, enumerators)               \
  DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value)

#endif // ENUMMACROS_H

