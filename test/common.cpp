#include "common.hpp"
#include <fstream>
#include <cstring>

namespace testing {
    namespace internal {
        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* abs_error_expr, glm::vec3 const& val1, glm::vec3 const& val2, double abs_error)
        {
            auto x_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.x, val2.x, abs_error);
            auto y_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.y, val2.y, abs_error);
            auto z_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.z, val2.z, abs_error);

            if (!x_res || !y_res || !z_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << abs_error_expr << ", where\n"
                       << expr1 << " evaluates to " << val1 << ",\n"
                       << expr2 << " evaluates to " << val2 << ", and\n"
                       << abs_error_expr << " evaluates to " << abs_error << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* abs_error_expr, glm::vec2 const& val1, glm::vec2 const& val2, double abs_error)
        {
            auto x_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.x, val2.x, abs_error);
            auto y_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.y, val2.y, abs_error);

            if (!x_res || !y_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << abs_error_expr << ", where\n"
                       << expr1 << " evaluates to " << val1 << ",\n"
                       << expr2 << " evaluates to " << val2 << ", and\n"
                       << abs_error_expr << " evaluates to " << abs_error << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* abs_error_expr, aabb const& val1, aabb const& val2, double abs_error)
        {
            auto a_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.min, val2.min, abs_error);
            auto b_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1.max, val2.max, abs_error);

            if (!a_res || !b_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << abs_error_expr << ", where\n"
                       << expr1 << " evaluates to [" << val1.min << "], [" << val1.max << "],\n"
                       << expr2 << " evaluates to [" << val2.min << "], [" << val2.max << "], and\n"
                       << abs_error_expr << " evaluates to " << abs_error << ".";
            }

            return AssertionSuccess();
        }

        AssertionResult DoubleNearPredFormat(const char* expr1, const char* expr2, const char* abs_error_expr, triangle const& val1, triangle const& val2, double abs_error)
        {
            auto a_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1[0], val2[0], abs_error);
            auto b_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1[1], val2[1], abs_error);
            auto c_res = DoubleNearPredFormat(expr1, expr2, abs_error_expr, val1[2], val2[2], abs_error);

            if (!a_res || !b_res || !c_res) {
                return AssertionFailure()
                       << "The difference between " << expr1 << " and " << expr2
                       << " exceeds " << abs_error_expr << ", where\n"
                       << expr1 << " evaluates to " << val1[0] << ", " << val1[1] << ", " << val1[2] << ",\n"
                       << expr2 << " evaluates to " << val2[0] << ", " << val2[1] << ", " << val2[2] << ", and\n"
                       << abs_error_expr << " evaluates to " << abs_error << ".";
            }

            return AssertionSuccess();
        }
    }
}
