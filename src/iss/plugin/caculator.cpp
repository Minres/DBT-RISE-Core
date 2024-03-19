#define BOOST_SPIRIT_NO_PREDEFINED_TERMINALS
// Define this to enable debugging
// #define BOOST_SPIRIT_QI_DEBUG

#if defined(_MSC_VER)
#pragma warning(disable : 4345)
#endif

#include "calculator.h"
#include <boost/foreach.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/phoenix/function.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/recursive_variant.hpp>
#include <util/ities.h>
#include <util/logging.h>

#include <iostream>
#include <list>
#include <string>
namespace iss {
namespace plugin {
namespace ast {
///////////////////////////////////////////////////////////////////////////
//  The AST
///////////////////////////////////////////////////////////////////////////
struct nil {};
struct signed_;
struct variable;
struct expression;

typedef boost::variant<nil, unsigned int, variable, boost::recursive_wrapper<signed_>, boost::recursive_wrapper<expression>> operand;

struct variable {
    variable(std::string const& name = "")
    : name(name) {}
    std::string name;
};

struct signed_ {
    char sign;
    operand operand_;
};

struct operation {
    char operator_;
    operand operand_;
};

struct expression {
    operand first;
    std::list<operation> rest;
};

// print function for debugging
inline std::ostream& operator<<(std::ostream& out, nil) {
    out << "nil";
    return out;
}
} // namespace ast
} // namespace plugin
} // namespace iss

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(iss::plugin::ast::variable,
        (std::string, name))
BOOST_FUSION_ADAPT_STRUCT(iss::plugin::ast::signed_,
        (char, sign)
        (iss::plugin::ast::operand, operand_))
BOOST_FUSION_ADAPT_STRUCT(iss::plugin::ast::operation,
        (char, operator_)
        (iss::plugin::ast::operand, operand_))
BOOST_FUSION_ADAPT_STRUCT(iss::plugin::ast::expression,
        (iss::plugin::ast::operand, first)
        (std::list<iss::plugin::ast::operation>, rest))
// clang-format on

namespace iss {
namespace plugin {
///////////////////////////////////////////////////////////////////////////
//  The Virtual Machine
///////////////////////////////////////////////////////////////////////////
enum byte_code { op_neg, op_add, op_sub, op_mul, op_div, op_int, op_var };

///////////////////////////////////////////////////////////////////////////
//  The Compiler
///////////////////////////////////////////////////////////////////////////
struct compiler {
    typedef void result_type;

    std::vector<int>& code;
    std::vector<std::function<unsigned(uint64_t)>>& var_accessors;
    uint32_t* reg_ptr;
    compiler(uint32_t* reg_ptr, std::vector<int>& code, std::vector<std::function<unsigned(uint64_t)>>& var_accessors)
    : code(code)
    , reg_ptr{reg_ptr}
    , var_accessors{var_accessors} {}

    void operator()(ast::nil) const { BOOST_ASSERT(0); }
    void operator()(unsigned int n) const {
        code.push_back(op_int);
        code.push_back(n);
    }

    void operator()(ast::variable const& v) const {
        code.push_back(op_var);
        code.push_back(var_accessors.size());
        auto pos = v.name.find('_');
        auto sep = v.name.find(':');
        auto upper = std::strtoul(v.name.substr(pos + 1, sep - pos - 1).c_str(), nullptr, 10);
        auto lower = std::strtoul(v.name.substr(sep + 1).c_str(), nullptr, 10);
        auto size = upper - lower + 1;
        auto mask = (1ULL << size) - 1;
        auto rp = reg_ptr;
        if(v.name[0] == 'X') {
            var_accessors.push_back([lower, mask, rp](uint64_t instr) -> unsigned { return *(rp + ((instr >> lower) & mask)); });
        } else if(v.name[0] == 's') {
            auto sign_mask = 1ULL << (size - 1);
            var_accessors.push_back([lower, mask, sign_mask, rp](uint64_t instr) -> unsigned {
                auto val = (instr >> lower) & mask;
                return (val & mask) | ((val & sign_mask) ? ~mask : 0);
            });
        } else {
            var_accessors.push_back([lower, mask](uint64_t instr) -> unsigned { return (instr >> lower) & mask; });
        }
    }

    void operator()(ast::operation const& x) const {
        boost::apply_visitor(*this, x.operand_);
        switch(x.operator_) {
        case '+':
            code.push_back(op_add);
            break;
        case '-':
            code.push_back(op_sub);
            break;
        case '*':
            code.push_back(op_mul);
            break;
        case '/':
            code.push_back(op_div);
            break;
        default:
            BOOST_ASSERT(0);
            break;
        }
    }

    void operator()(ast::signed_ const& x) const {
        boost::apply_visitor(*this, x.operand_);
        switch(x.sign) {
        case '-':
            code.push_back(op_neg);
            break;
        case '+':
            break;
        default:
            BOOST_ASSERT(0);
            break;
        }
    }

    void operator()(ast::expression const& x) const {
        boost::apply_visitor(*this, x.first);
        BOOST_FOREACH(ast::operation const& oper, x.rest) {
            (*this)(oper);
        }
    }
};

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
using boost::phoenix::function;

///////////////////////////////////////////////////////////////////////////////
//  The error handler
///////////////////////////////////////////////////////////////////////////////
struct error_handler_ {
    template <typename, typename, typename> struct result { typedef void type; };

    template <typename Iterator> void operator()(qi::info const& what, Iterator err_pos, Iterator last) const {
        CPPLOG(ERR) << "Expecting " << what                      // what failed?
                    << " here: \"" << std::string(err_pos, last) // iterators to error-pos, end
                    << "\"" << std::endl;
    }
};

function<error_handler_> const error_handler = error_handler_();

///////////////////////////////////////////////////////////////////////////////
//  The calculator grammar
///////////////////////////////////////////////////////////////////////////////
template <typename Iterator> struct grammar : qi::grammar<Iterator, ast::expression(), ascii::space_type> {
    grammar()
    : grammar::base_type(expression) {
        qi::char_type char_;
        qi::uint_type uint_;
        qi::_2_type _2;
        qi::_3_type _3;
        qi::_4_type _4;
        qi::raw_type raw;
        qi::lexeme_type lexeme;
        qi::alpha_type alpha;
        qi::alnum_type alnum;

        using qi::fail;
        using qi::on_error;
        expression = term >> *((char_('+') > term) | (char_('-') > term));
        term = factor >> *((char_('*') > factor) | (char_('/') > factor));
        identifier = raw[lexeme[(alpha) >> *(alnum | '_' | ':')]];
        factor = uint_ | identifier | '(' > expression > ')' | (char_('-') > factor) | (char_('+') > factor);
        // Debugging and error handling and reporting support.
        BOOST_SPIRIT_DEBUG_NODES((expression)(term)(identifier)(factor));
        // Error handling
        on_error<fail>(expression, error_handler(_4, _3, _2));
    }

    qi::rule<Iterator, ast::expression(), ascii::space_type> expression;
    qi::rule<Iterator, ast::expression(), ascii::space_type> term;
    qi::rule<Iterator, ast::operand(), ascii::space_type> factor;
    qi::rule<Iterator, std::string(), ascii::space_type> identifier;
};

calculator::calculator(uint32_t* reg_base_ptr, const std::string& formula)
: reg_base_ptr{reg_base_ptr} {
    using iterator_type = std::string::const_iterator;
    grammar<iterator_type> grammar;
    ast::expression expression;
    compiler compile{reg_base_ptr, byte_code, var_accessors};
    std::string::const_iterator iter = formula.begin();
    std::string::const_iterator end = formula.end();
    boost::spirit::ascii::space_type space;
    bool r = boost::spirit::qi::phrase_parse(iter, end, grammar, space, expression);
    if(r && iter == end) {
        compile(expression);
        error = "";
    } else
        error = std::string(iter, end);
}

calculator::~calculator() = default;

calculator::calculator(const calculator&) = default;

unsigned calculator::operator()(uint64_t instr) {
    std::vector<int>::const_iterator pc = byte_code.begin();
    stack_ptr = stack.begin();
    while(pc != byte_code.end()) {
        switch(*pc++) {
        case op_neg:
            stack_ptr[-1] = -stack_ptr[-1];
            break;
        case op_add:
            --stack_ptr;
            stack_ptr[-1] += stack_ptr[0];
            break;
        case op_sub:
            --stack_ptr;
            stack_ptr[-1] -= stack_ptr[0];
            break;
        case op_mul:
            --stack_ptr;
            stack_ptr[-1] *= stack_ptr[0];
            break;
        case op_div:
            --stack_ptr;
            stack_ptr[-1] /= stack_ptr[0];
            break;
        case op_var:
            *stack_ptr = var_accessors[*pc](instr);
            stack_ptr++;
            pc++;
            break;
        case op_int:
            *stack_ptr = *pc;
            stack_ptr++;
            pc++;
            break;
        }
    }
    return stack_ptr[-1];
}
} // namespace plugin
} // namespace iss
