#include "aerodynamics/LiftCurveModel.hpp"
#include "liteaerosim.pb.h"
#include <stdexcept>

LiftCurveModel::LiftCurveModel(const LiftCurveParams& params) : _p(params) {
    computeCoefficients();
}

void LiftCurveModel::computeCoefficients() {
    // ── Precondition checks ───────────────────────────────────────────────────
    if (_p.cl_sep > _p.cl_max) {
        throw std::invalid_argument(
            "LiftCurveModel: cl_sep must not exceed cl_max");
    }
    if (_p.cl_sep_neg < _p.cl_min) {
        throw std::invalid_argument(
            "LiftCurveModel: cl_sep_neg must not be less than cl_min");
    }

    // ── Positive stall ────────────────────────────────────────────────────────
    // Downward-opening parabola with vertex at (_alpha_peak, cl_max).
    // C¹ joined to the linear model at _alpha_star.
    _alpha_peak = _p.cl_max / _p.cl_alpha + _p.delta_alpha_stall / 2.0f;
    _alpha_star  = _alpha_peak - _p.delta_alpha_stall;

    _a2 = -_p.cl_alpha / (2.0f * _p.delta_alpha_stall);
    _a1 = -2.0f * _a2 * _alpha_peak;
    _a0 = _a2 * _alpha_peak * _alpha_peak + _p.cl_max;

    // α_sep: descending side where C_L = cl_sep
    _alpha_sep = _alpha_peak + std::sqrt((_p.cl_sep - _p.cl_max) / _a2);

    // ── Negative stall ────────────────────────────────────────────────────────
    // Upward-opening parabola with vertex at (_alpha_peak_neg, cl_min).
    // C¹ joined to the linear model at _alpha_star_neg.
    _alpha_peak_neg = _p.cl_min / _p.cl_alpha - _p.delta_alpha_stall_neg / 2.0f;
    _alpha_star_neg  = _alpha_peak_neg + _p.delta_alpha_stall_neg;

    _a2n = _p.cl_alpha / (2.0f * _p.delta_alpha_stall_neg);
    _a1n = -2.0f * _a2n * _alpha_peak_neg;
    _a0n = _a2n * _alpha_peak_neg * _alpha_peak_neg + _p.cl_min;

    // α_sep_neg: ascending side (going more negative) where C_L = cl_sep_neg
    _alpha_sep_neg = _alpha_peak_neg - std::sqrt((_p.cl_sep_neg - _p.cl_min) / _a2n);
}

float LiftCurveModel::evaluate(float alpha_rad) const {
    if (alpha_rad < _alpha_sep_neg) {
        return _p.cl_sep_neg;
    }
    if (alpha_rad < _alpha_star_neg) {
        return _a2n * alpha_rad * alpha_rad + _a1n * alpha_rad + _a0n;
    }
    if (alpha_rad <= _alpha_star) {
        return _p.cl_alpha * alpha_rad;
    }
    if (alpha_rad <= _alpha_sep) {
        return _a2 * alpha_rad * alpha_rad + _a1 * alpha_rad + _a0;
    }
    return _p.cl_sep;
}

float LiftCurveModel::derivative(float alpha_rad) const {
    if (alpha_rad < _alpha_sep_neg) {
        return 0.0f;
    }
    if (alpha_rad < _alpha_star_neg) {
        return 2.0f * _a2n * alpha_rad + _a1n;
    }
    if (alpha_rad <= _alpha_star) {
        return _p.cl_alpha;
    }
    if (alpha_rad <= _alpha_sep) {
        return 2.0f * _a2 * alpha_rad + _a1;
    }
    return 0.0f;
}

float LiftCurveModel::alphaPeak()    const { return _alpha_peak; }
float LiftCurveModel::alphaStar()    const { return _alpha_star; }
float LiftCurveModel::alphaTrough()  const { return _alpha_peak_neg; }
float LiftCurveModel::alphaStarNeg() const { return _alpha_star_neg; }
float LiftCurveModel::alphaSep()     const { return _alpha_sep; }
float LiftCurveModel::alphaSepNeg()  const { return _alpha_sep_neg; }
float LiftCurveModel::clAlpha()      const { return _p.cl_alpha; }
float LiftCurveModel::clSep()        const { return _p.cl_sep; }
float LiftCurveModel::clSepNeg()     const { return _p.cl_sep_neg; }

LiftCurveSegment LiftCurveModel::classify(float alpha_rad) const {
    if (alpha_rad < _alpha_sep_neg)   return LiftCurveSegment::FullySeparatedNegative;
    if (alpha_rad < _alpha_peak_neg)  return LiftCurveSegment::PostStallNegative;
    if (alpha_rad < _alpha_star_neg)  return LiftCurveSegment::IncipientStallNegative;
    if (alpha_rad <= _alpha_star)     return LiftCurveSegment::Linear;
    if (alpha_rad <= _alpha_peak)     return LiftCurveSegment::IncipientStallPositive;
    if (alpha_rad <= _alpha_sep)      return LiftCurveSegment::PostStallPositive;
    return LiftCurveSegment::FullySeparatedPositive;
}

// ── Serialization ─────────────────────────────────────────────────────────────

nlohmann::json LiftCurveModel::serializeJson() const {
    return {
        {"schema_version",        1},
        {"type",                  "LiftCurveModel"},
        {"cl_alpha",              _p.cl_alpha},
        {"cl_max",                _p.cl_max},
        {"cl_min",                _p.cl_min},
        {"delta_alpha_stall",     _p.delta_alpha_stall},
        {"delta_alpha_stall_neg", _p.delta_alpha_stall_neg},
        {"cl_sep",                _p.cl_sep},
        {"cl_sep_neg",            _p.cl_sep_neg},
    };
}

LiftCurveModel LiftCurveModel::deserializeJson(const nlohmann::json& j) {
    if (j.at("schema_version").get<int>() != 1) {
        throw std::runtime_error("LiftCurveModel::deserializeJson: unsupported schema_version");
    }
    LiftCurveParams p;
    p.cl_alpha              = j.at("cl_alpha").get<float>();
    p.cl_max                = j.at("cl_max").get<float>();
    p.cl_min                = j.at("cl_min").get<float>();
    p.delta_alpha_stall     = j.at("delta_alpha_stall").get<float>();
    p.delta_alpha_stall_neg = j.at("delta_alpha_stall_neg").get<float>();
    p.cl_sep                = j.at("cl_sep").get<float>();
    p.cl_sep_neg            = j.at("cl_sep_neg").get<float>();
    return LiftCurveModel(p);
}

std::vector<uint8_t> LiftCurveModel::serializeProto() const {
    las_proto::LiftCurveParams proto;
    proto.set_schema_version(1);
    proto.set_cl_alpha(_p.cl_alpha);
    proto.set_cl_max(_p.cl_max);
    proto.set_cl_min(_p.cl_min);
    proto.set_delta_alpha_stall(_p.delta_alpha_stall);
    proto.set_delta_alpha_stall_neg(_p.delta_alpha_stall_neg);
    proto.set_cl_sep(_p.cl_sep);
    proto.set_cl_sep_neg(_p.cl_sep_neg);
    const std::string s = proto.SerializeAsString();
    return std::vector<uint8_t>(s.begin(), s.end());
}

LiftCurveModel LiftCurveModel::deserializeProto(const std::vector<uint8_t>& bytes) {
    las_proto::LiftCurveParams proto;
    if (!proto.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("LiftCurveModel::deserializeProto: failed to parse bytes");
    }
    if (proto.schema_version() != 1) {
        throw std::runtime_error("LiftCurveModel::deserializeProto: unsupported schema_version");
    }
    LiftCurveParams p;
    p.cl_alpha              = proto.cl_alpha();
    p.cl_max                = proto.cl_max();
    p.cl_min                = proto.cl_min();
    p.delta_alpha_stall     = proto.delta_alpha_stall();
    p.delta_alpha_stall_neg = proto.delta_alpha_stall_neg();
    p.cl_sep                = proto.cl_sep();
    p.cl_sep_neg            = proto.cl_sep_neg();
    return LiftCurveModel(p);
}
