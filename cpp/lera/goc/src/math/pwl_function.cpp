//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/math/pwl_function.h"

#include "goc/print/print_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/string/string_utils.h"
#include "goc/math/number_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{

PWLFunction PWLFunction::ConstantFunction(double a, Interval domain)
{
    PWLFunction f;
    f.AddPiece(LinearFunction(Point2D(domain.left, a), Point2D(domain.right, a)));
    return f;
}

PWLFunction PWLFunction::IdentityFunction(Interval domain)
{
    PWLFunction f;
    f.AddPiece(LinearFunction(Point2D(domain.left, domain.left), Point2D(domain.right, domain.right)));
    return f;
}

PWLFunction::PWLFunction()
{
    domain_ = image_ = {INFTY, -INFTY};
}

PWLFunction::PWLFunction(const std::vector<LinearFunction>& pieces) : PWLFunction()
{
    for (auto& p: pieces) AddPiece(p);
}

PWLFunction::PWLFunction(
    const vector<double>& breakpoints,
    const vector<double>& values
) {
    if (breakpoints.empty() || values.empty() || breakpoints.size() != values.size()) {
        throw invalid_argument("Breakpoints and values must be non-empty and of the same size.");
    }
    
    #ifndef NDEBUG
    // Check if breakpoints are sorted and unique
    for (size_t i = 1; i < breakpoints.size(); ++i) {
        if (epsilon_smaller_equal(breakpoints[i], breakpoints[i-1])) {
            throw invalid_argument("Breakpoints must be sorted and unique.");
        }
    }
    #endif

    pieces_.reserve(breakpoints.size() - 1);
    for (size_t i = 0; i < breakpoints.size() - 1; ++i)
    {
        double x_left = breakpoints[i];
        double x_right = breakpoints[i + 1];
        double y_left = values[i];
        double y_right = values[i + 1];
        
        AddPiece(LinearFunction(
            Point2D(x_left, y_left),
            Point2D(x_right, y_right)
        ));
    }

    #ifndef NDEBUG
    // Check if the function is normalized.
    if (!check_invariant()) {
        throw runtime_error("PWLFunction invariant violated after construction.");
    }
    #endif
}

void PWLFunction::AddPiece(const LinearFunction& piece)
{
    // If this piece is a continuation of the last piece, then we need to merge them into one piece to have the
    // function normalized.
    bool is_continuation_of_last_piece = false;
    if (!pieces_.empty())
    {
        if (epsilon_equal(pieces_.back().domain.right, piece.domain.left))
        {
            double right_val = pieces_.back().Value(max(dom(pieces_.back())));
            double left_val = piece.Value(min(dom(piece)));
            is_continuation_of_last_piece = epsilon_equal(left_val, right_val) && (pieces_.back().domain.IsPoint() || epsilon_equal(pieces_.back().slope, piece.slope));
        }
    }
    
    if (!is_continuation_of_last_piece)
    {
        // Add the new piece if it is not a continuation.
        pieces_.push_back(piece);
    }
    else
    {
        // Merge pieces to have the function normalized.
        pieces_.back().domain.right = piece.domain.right;
        pieces_.back().image.left = min(pieces_.back().image.left, piece.image.left);
        pieces_.back().image.right = max(pieces_.back().image.right, piece.image.right);
        pieces_.back().slope = piece.slope;
        pieces_.back().intercept = piece.intercept;
    }
    
    // Update domain and image.
    if (domain_.left == INFTY) domain_.left = piece.domain.left;
    domain_.right = piece.domain.right;
    image_.left = min(image_.left, piece.image.left);
    image_.right = max(image_.right, piece.image.right);
}

void PWLFunction::PopPiece()
{
    pieces_.pop_back();
    domain_ = Empty() ? Interval(INFTY, -INFTY) : Interval(domain_.left, pieces_.back().domain.right);
    UpdateImage();
}

bool PWLFunction::Empty() const
{
    return pieces_.empty();
}

int PWLFunction::PieceCount() const
{
    return pieces_.size();
}

const vector<LinearFunction>& PWLFunction::Pieces() const
{
    return pieces_;
}

const LinearFunction& PWLFunction::Piece(int i) const
{
    return pieces_[i];
}

const LinearFunction& PWLFunction::operator[](int i) const
{
    return pieces_[i];
}

const LinearFunction& PWLFunction::FirstPiece() const
{
    return pieces_.front();
}

const LinearFunction& PWLFunction::LastPiece() const
{
    return pieces_.back();
}

int PWLFunction::PieceIncluding(double x) const
{
    // if x is outside the Domain(), throw exception.
    if (epsilon_bigger(domain_.left, x) || epsilon_smaller(domain_.right, x))
    {
        fail("PWLFunction::Value(" + STR(x) +") failed, because domain is " + STR(Domain()));
        return -1;
    }
    
    // Look for a piece that include x in their domain.
    for (int i = pieces_.size()-1; i >= 0; --i)
        if (pieces_[i].domain.Includes(x))
            return i;
    
    // The function is not continuous and x is not in the domain of any piece.
    fail("PWLFunction::Value(" + STR(x) +") failed, because x is not inside the domain of its pieces.");
    return -1;
}

Interval PWLFunction::Domain() const
{
    return domain_;
}

Interval PWLFunction::Image() const
{
    return image_;
}

double PWLFunction::Value(double x) const
{
    // if x is outside the Domain(), throw exception.
    if (epsilon_bigger(domain_.left, x) || epsilon_smaller(domain_.right, x))
    {
        fail("PWLFunction::Value(" + STR(x) +") failed, because domain is " + STR(Domain()));
        return -1;
    }

    // Left-continuous evaluation (M5.9): return the FIRST piece (front-to-back)
    // whose domain includes x. At the abscissa of a value jump this is the piece
    // ending at x from the left, so Value returns the pre-jump (lower) value —
    // the checker's authoritative convention, and the one every step-aware
    // primitive below relies on. For a continuous function all pieces meeting at
    // a breakpoint agree, so this is identical (modulo the pre-existing
    // slope*x+intercept ulp) to the previous back-to-front scan; it only
    // disambiguates genuine verticals (a zero-width piece is ordered after the
    // left piece that ends at its abscissa, so it is never selected for x).
    for (int i = 0; i < (int) pieces_.size(); ++i)
        if (pieces_[i].domain.Includes(x))
            return pieces_[i].Value(x);

    // The function is not continuous and x is not in the domain of any piece.
    fail("PWLFunction::Value(" + STR(x) +") failed, because x is not inside the domain of its pieces.");
    return -1;
}

double PWLFunction::operator()(double x) const
{
    return Value(x);
}

double PWLFunction::PreValue(double y) const
{
    if (epsilon_bigger(image_.left, y) || epsilon_smaller(image_.right, y))
    {
        fail("PWLFunction::PreValue(" + STR(y) +") failed, because image is " + STR(Image()));
        return -1;
    }
    
    // Look for a piece that include x in their domain.
    for (int i = (int)pieces_.size()-1; i >= 0; --i)
        if (pieces_[i].image.Includes(y))
            return pieces_[i].PreValue(y);
    
    // The function is not continuous and x is not in the domain of any piece.
    fail("PWLFunction::PreValue(" + STR(y) +") failed, because y is not inside the domain of its pieces.");
    return -1;
}

PWLFunction PWLFunction::Compose(const PWLFunction& g) const
{
    PWLFunction fog; // fog is the composition of this function (f) and g, i.e. fog(x) == f(g(x)).
    
    auto& f = *this;
    int i = 0;
    for (int j = 0; j < g.PieceCount(); ++j)
    {
        // Put i inside bounds.
        i = max(0, min(f.PieceCount()-1, i));
        
        // If g[j] is constant, image is a single y = min(img(g[j])) = max(img(g[j])).
        if (epsilon_equal(g[j].slope, 0.0))
        {
            double y = min(img(g[j]));
            
            // Find (unique) piece f[i] with img(g[j]) \subseteq dom(f[i]) if exists.
            while (i < f.PieceCount()-1 && epsilon_smaller(max(dom(f[i])), y)) ++i;
            while (i > 0 && epsilon_bigger(min(dom(f[i])), y)) --i;
            
            // If found f[i] such that dom(f[i]) \cap img(g[j]) \neq \emptyset, add the piece to fog.
            if (i >= 0 && i < f.PieceCount() && dom(f[i]).Includes(y))
                fog.AddPiece(LinearFunction({min(dom(g[j])), f[i](y)}, {max(dom(g[j])), f[i](y)}));
        }
        // If g[j] is increasing.
        else if (epsilon_bigger(g[j].slope, 0.0))
        {
            // Find first piece f[i] such that max(dom(f[i])) >= min(img(g[j])).
            while (i > 0 && epsilon_bigger_equal(max(dom(f[i-1])), min(img(g[j])))) --i;
            while (i < f.PieceCount() && epsilon_smaller(max(dom(f[i])), min(img(g[j])))) ++i;
            
            // For each piece f[i] such that dom(f[i]) \cap img(g[j]) \neq \emptyset
            for (; i < f.PieceCount() && dom(f[i]).Intersects(img(g[j])); ++i)
            {
                // Find intersection.
                Interval inter = dom(f[i]).Intersection(img(g[j]));
                double left = g[j].PreValue(inter.left), right = g[j].PreValue(inter.right);
                fog.AddPiece(LinearFunction({left, f[i](inter.left)}, {right, f[i](inter.right)}));
            }
        }
        // If g[j] is decreasing.
        else if (epsilon_smaller(g[j].slope, 0.0))
        {
            // Find last piece f[i] such that min(dom(f[i])) <= max(img(g[j])).
            while (i < f.PieceCount()-1 && epsilon_smaller_equal(min(dom(f[i+1])), max(img(g[j])))) ++i;
            while (i > 0 && epsilon_bigger(min(dom(f[i])), max(img(g[j])))) --i;
            
            // For each piece f[i] such that dom(f[i]) \cap img(g[j]) \neq \emptyset
            for (; i >= 0 && i < f.PieceCount() && dom(f[i]).Intersects(img(g[j])); --i)
            {
                // Find intersection.
                Interval inter = dom(f[i]).Intersection(img(g[j]));
                double left = g[j].PreValue(inter.right), right = g[j].PreValue(inter.left);
                fog.AddPiece(LinearFunction({left, f[i](inter.right)}, {right, f[i](inter.left)}));
            }
        }
    }
    return fog;
}

PWLFunction PWLFunction::Inverse() const
{
    if (Empty()) return PWLFunction();

    // Faithful breakpoint list (xs, ys). A vertical piece (a value jump) is
    // read from its IMAGE endpoints — Value returns only the left-continuous
    // (lower) value, but image = [lo, hi] retains the jump. A non-first piece's
    // left endpoint coincides with the previous piece's right endpoint, so we
    // append only right endpoints.
    std::vector<double> xs, ys;
    xs.reserve(pieces_.size() + 1);
    ys.reserve(pieces_.size() + 1);
    for (const LinearFunction& p : pieces_)
    {
        double xl = p.domain.left, xr = p.domain.right, yl, yr;
        if (p.is_vertical()) { yl = p.image.left; yr = p.image.right; }
        else { yl = p.slope * xl + p.intercept; yr = p.slope * xr + p.intercept; }
        if (xs.empty()) { xs.push_back(xl); ys.push_back(yl); }
        xs.push_back(xr); ys.push_back(yr);
    }

    // The inverse of a NON-DECREASING PWL is the exact coordinate swap
    // (xs <-> ys): both vectors are non-decreasing, a value jump (duplicate x)
    // becomes a plateau (duplicate y) and vice versa, with no numerical error.
    // This is the operation the 1e-3 mollifier existed to avoid; arrival /
    // departure functions (the only callers) are non-decreasing. For a
    // non-monotone function we fall back to goc's max{x : f(x)=y} behavior.
    bool non_decreasing = true;
    for (size_t i = 1; i < ys.size(); ++i)
        if (epsilon_smaller(ys[i], ys[i - 1])) { non_decreasing = false; break; }

    if (!non_decreasing)
    {
        PWLFunction g;
        for (auto& p: Pieces()) g = Max(g, PWLFunction({p.Inverse()}));
        return g;
    }

    PWLFunction inv;
    for (size_t i = 0; i + 1 < ys.size(); ++i)
        inv.AddPiece(LinearFunction(Point2D(ys[i], xs[i]), Point2D(ys[i + 1], xs[i + 1])));
    return inv;
}

PWLFunction PWLFunction::RestrictDomain(const Interval& domain) const
{
    PWLFunction f;
    for (auto& p: Pieces())
    {
        if (!domain.Intersects(p.domain)) continue;
        f.AddPiece(p.RestrictDomain(domain));
    }
    return f;
}

PWLFunction PWLFunction::RestrictImage(const Interval& image) const
{
    PWLFunction f;
    for (auto& p: Pieces())
    {
        if (!image.Intersects(p.image)) continue;
        f.AddPiece(p.RestrictImage(image));
    }
    return f;
}

bool PWLFunction::check_invariant() const
{
    if (pieces_.empty()) return domain_ == Interval(INFTY, -INFTY) && image_ == Interval(INFTY, -INFTY);
    
    // Check that the domain is the union of the pieces domains.
    Interval domain = pieces_.front().domain;
    for (auto& p: pieces_) domain = domain.Union(p.domain);
    if (domain != domain_) return false;
    
    // Check that the image is the union of the pieces images.
    Interval image = pieces_.front().image;
    for (auto& p: pieces_) image = image.Union(p.image);
    if (image != image_) return false;
    
    return true;
}

bool PWLFunction::check_normalization() const {
    if (pieces_.empty()) return domain_ == Interval(INFTY, -INFTY) && image_ == Interval(INFTY, -INFTY);
    
    
    for (size_t i = 1; i < pieces_.size(); ++i)
    {
        // Check that the adjacent pieces have adjacent domains.
        if (epsilon_different(pieces_[i].domain.left, pieces_[i-1].domain.right) ||
            epsilon_different(pieces_[i].Value(pieces_[i].domain.left), pieces_[i-1].Value(pieces_[i-1].domain.right)))
        {
            std::cerr << "PWLFunction::check_normalization: pieces have non-adjacent domains "
                        << pieces_[i-1].domain << " and " << pieces_[i].domain 
                        << " at indices " << i-1 << " and " << i
                        << ". Values are " << pieces_[i-1].Value(pieces_[i-1].domain.right)
                        << " and " << pieces_[i].Value(pieces_[i].domain.left)
                        << std::endl;
            return false;
        }
        // Check that adjascent domains are of increasing values
        if (epsilon_smaller_equal(pieces_[i].domain.right, pieces_[i-1].domain.left))
        {
            std::cerr << "PWLFunction::check_normalization: pieces have non-increasing domains "
                        << pieces_[i-1].domain << " and " << pieces_[i].domain 
                        << " at indices " << i-1 << " and " << i
                        << std::endl;
            return false;
        }
    }

    // Check that no piece is the continuation of the previous piece.
    for (size_t i = 1; i < pieces_.size(); ++i)
    {
        if (epsilon_equal(pieces_[i-1].domain.right, pieces_[i].domain.left) &&
            epsilon_equal(pieces_[i-1].Value(pieces_[i-1].domain.right), pieces_[i].Value(pieces_[i].domain.left)) &&
            epsilon_equal(pieces_[i-1].slope, pieces_[i].slope))
        {   
            std::cerr << "PWLFunction::check_normalization: pieces are continuations of each other "
                        << pieces_[i-1].domain << " and " << pieces_[i].domain 
                        << " at indices " << i-1 << " and " << i
                        << ". Slopes are " << pieces_[i-1].slope << " and " << pieces_[i].slope
                        << std::endl;
            return false;
        }
    }
    
    return true;
}

bool PWLFunction::check_continuity() const 
{
	if (pieces_.empty()) return domain_ == Interval(INFTY, -INFTY) && image_ == Interval(INFTY, -INFTY);

	// Check that the pieces are continuous.
	for (size_t i = 1; i < pieces_.size(); ++i)
	{
		if (!epsilon_equal(pieces_[i-1].domain.right, pieces_[i].domain.left) ||
			!epsilon_equal(pieces_[i-1].Value(pieces_[i-1].domain.right), pieces_[i].Value(pieces_[i].domain.left)))
		{
			std::cerr << "PWLFunction::check_continuity: discontinuity found between pieces "
				<< " at indices " << i-1 << " and " << i
				<< "piece n°" << i - 1 << " right endpoint: (" << pieces_[i-1].domain.right << ", "
				<< pieces_[i-1].Value(pieces_[i-1].domain.right) << ") and "
				<< "piece n°" << i << " left endpoint: (" << pieces_[i].domain.left << ", "
				<< pieces_[i].Value(pieces_[i].domain.left) << ")"
				<< std::endl;
			return false;
		}
	}

	return true;
}

const std::pair<std::vector<double>, std::vector<double>> PWLFunction::copy_breakpoints_and_values() const
{
    std::vector<double> breakpoints;
    std::vector<double> values;

    // If there are no pieces, return empty vectors.
    if (pieces_.empty())
    {
        return {breakpoints, values}; // return empty vectors
    }

	#ifndef NDEBUG
	// Ensure continuity of the pieces.
	if (!check_continuity()) {
		Print(std::cerr);
		throw std::runtime_error("PWLFunction::copy_breakpoints_and_values: discontinuity found in the pieces.");
	}
	#endif

    for (auto& p: pieces_)
    {
        // Note that we expect the pieces to be over a continuous domain
        // so adjascent pieces share breakpoints.
        if (breakpoints.empty())
        {
            breakpoints.push_back(p.domain.left);
            breakpoints.push_back(p.domain.right);
            values.push_back(p.Value(p.domain.left));
            values.push_back(p.Value(p.domain.right));
        }
        else
        {
            // check that the last breakpoint matches the first of the current piece
            // else throw an error.
			// #ifndef NDEBUG
            // if (!epsilon_equal(breakpoints.back(), p.domain.left) ||
            //     !epsilon_equal(values.back(), p.Value(p.domain.left)))
            // {
            //     std::cerr << "PWLFunction::get_breakpoints_and_values: discontinuity found between pieces. "
            //                 << "Last point: (" << breakpoints.back() << ", "
            //                 << values.back() << ") and "
            //                 << "first point of current piece: (" << p.domain.left << ", "
            //                 << p.Value(p.domain.left) << ")" << std::endl;
            //     Print(std::cerr);
            //     throw std::runtime_error("PWLFunction::get_breakpoints_and_values: discontinuity found between pieces.");
            // }
			// #endif
            // add the right endpoint of the current piece
            breakpoints.push_back(p.domain.right);
            values.push_back(p.Value(p.domain.right));
        }
    }
    return {breakpoints, values};
}

void PWLFunction::Print(std::ostream& os) const
{
    os << "[";
    for (int i = 0; i < PieceCount(); ++i)
    {
        if (i > 0) os << ", ";
        os << Piece(i);
    }
    os << "]";
}

bool PWLFunction::operator==(const PWLFunction& f) const
{
    if (PieceCount() != f.PieceCount()) return false;
    for (int i = 0; i < PieceCount(); ++i) if (Piece(i) != f.Piece(i)) return false;
    return true;
}

bool PWLFunction::operator!=(const PWLFunction& f) const
{
    return !(*this == f);
}

void PWLFunction::UpdateImage()
{
    image_ = {INFTY, -INFTY};
    for (auto& p: pieces_)
    {
        image_.left = min(image_.left, p.image.left);
        image_.right = max(image_.right, p.image.right);
    }
}

void from_json(const json& j, PWLFunction& f)
{
    for (auto& p: j) f.AddPiece(p);
}

void to_json(json& j, const PWLFunction& f)
{
    j = vector<LinearFunction>();
    for (auto& p: f.Pieces()) j.push_back(p);
}

string to_string(const PWLFunction& f)
{
    ostringstream msg;
    msg << "PWLFunction" << endl;

    // print domain and image intervals
    msg << setw(14) << f.Domain() << setw(14) << f.Image() << endl;
    
    vector<double> xs;
    vector<double> ys;
    for (size_t i = 0; i < f.Pieces().size(); ++i)
    {
        auto&p = f.Pieces()[i];
        xs.push_back(p.domain.left);
        xs.push_back(p.domain.right);
        ys.push_back(p.image.left);
        ys.push_back(p.image.right);
        
        if (i >= f.Pieces().size() - 1) // if last piece
        {
            msg << setw(26) << setprecision(6) << fixed << p.domain.left << setw(26) << setprecision(6) << fixed << p.image.left << endl;
            msg << setw(26) << setprecision(6) << fixed << p.domain.right << setw(26) << setprecision(6) << fixed << p.image.right << endl; 
        }
        else
        {
            msg << setw(26) << setprecision(6) << fixed << p.domain.left << setw(26) << setprecision(6) << fixed << p.image.left << endl;
        }
    }
    // print vectors, using goc/print/print_utils.h '<<'
    print_padded_vectors(msg, xs, ys);
    // msg << f.DomainBreakpoints();
    // msg << f.ImageBreakpoints();
    return msg.str();
}

PWLFunction operator+(const PWLFunction& f, const PWLFunction& g)
{
    PWLFunction h;
    int i = 0, j = 0;
    while (i < f.PieceCount() && j < g.PieceCount())
    {
        auto& pf = f.Piece(i), &pg = g.Piece(j);
        if (pf.domain.Intersects(pg.domain))
        {
            double left = max(pf.domain.left, pg.domain.left), right = min(pf.domain.right, pg.domain.right);
            if (pf.is_vertical() || pg.is_vertical())
            {
                // M5.9: a value jump in either operand -> a value jump in the sum
                // at the same abscissa x0 (== left == right). The sum's image is
                // the jump's image shifted by the other (continuous) function's
                // value there; if both jump, the two images add. Value() on a
                // vertical returns only its lower endpoint, so we add the image
                // endpoints directly to keep the jump.
                double lo_f = pf.is_vertical() ? pf.image.left  : pf.Value(left);
                double hi_f = pf.is_vertical() ? pf.image.right : pf.Value(left);
                double lo_g = pg.is_vertical() ? pg.image.left  : pg.Value(left);
                double hi_g = pg.is_vertical() ? pg.image.right : pg.Value(left);
                h.AddPiece(LinearFunction({left, lo_f + lo_g}, {left, hi_f + hi_g}));
            }
            else
                h.AddPiece(LinearFunction({left, pf.Value(left)+pg.Value(left)}, {right, pf.Value(right)+pg.Value(right)}));
        }
        if (epsilon_equal(pf.domain.right, pg.domain.right)) { ++i; ++j; }
        else if (epsilon_smaller(pf.domain.right, pg.domain.right)) { ++i; }
        else { ++j; }
    }
    return h;
}

PWLFunction operator-(const PWLFunction& f, const PWLFunction& g)
{
    return f + g * -1.0;
}

PWLFunction operator*(const PWLFunction& f, const PWLFunction& g)
{
    PWLFunction h;
    int i = 0, j = 0;
    while (i < f.PieceCount() && j < g.PieceCount())
    {
        auto& pf = f.Piece(i), &pg = g.Piece(j);
        if (pf.domain.Intersects(pg.domain))
        {
            double left = max(pf.domain.left, pg.domain.left), right = min(pf.domain.right, pg.domain.right);
            h.AddPiece(LinearFunction({left, pf.Value(left)*pg.Value(left)}, {right, pf.Value(right)*pg.Value(right)}));
        }
        if (epsilon_equal(pf.domain.right, pg.domain.right)) { ++i; ++j; }
        else if (epsilon_smaller(pf.domain.right, pg.domain.right)) { ++i; }
        else { ++j; }
    }
    return h;
}

PWLFunction operator+(const PWLFunction& f, double a)
{
    return f + PWLFunction::ConstantFunction(a, f.Domain());
}

PWLFunction operator+(double a, const PWLFunction& f)
{
    return f + a;
}

PWLFunction operator-(const PWLFunction& f, double a)
{
    return f + (-1 * a);
}

PWLFunction operator-(double a, const PWLFunction& f)
{
    return f * -1.0 + a;
}

PWLFunction operator*(const PWLFunction& f, double a)
{
    return f * PWLFunction::ConstantFunction(a, f.Domain());
}

PWLFunction operator*(double a, const PWLFunction& f)
{
    return f * a;
}

PWLFunction Max(const PWLFunction& f_orig, const PWLFunction& g_orig)
{
    PWLFunction f = f_orig, g = g_orig;
    PWLFunction h;
    int i = 0, j = 0;
    while (i < f.PieceCount() && j < g.PieceCount())
    {
        auto& pf = (LinearFunction&)f.Piece(i);
        auto& pg = (LinearFunction&)g.Piece(j);
        // If pf has a part before pg.
        if (epsilon_smaller(pf.domain.left, pg.domain.left))
        {
            double l = pf.domain.left, r = min(pf.domain.right, pg.domain.left);
            h.AddPiece(LinearFunction(Point2D(l, pf.Value(l)), Point2D(r, pf.Value(r))));
            pf.domain.left = r;
            pf.image.left = pf.Value(r);
            if (epsilon_equal(r, pf.domain.right)) ++i;
        }
        else if (epsilon_smaller(pg.domain.left, pf.domain.left))
        {
            double l = pg.domain.left, r = min(pg.domain.right, pf.domain.left);
            h.AddPiece(LinearFunction(Point2D(l, pg.Value(l)), Point2D(r, pg.Value(r))));
            pg.domain.left = r;
            pg.image.left = pg.Value(r);
            if (epsilon_equal(r, pg.domain.right)) ++j;
        }
        else if (epsilon_equal(pf.domain.left, pg.domain.left))
        {
            double inter = pf.Intersection(pg);
            double l = pf.domain.left;
            double r = min(pf.domain.right, pg.domain.right);
            if (epsilon_bigger(inter, l) && epsilon_smaller(inter, r)) r = inter;
            h.AddPiece(LinearFunction(Point2D(l, max(pf.Value(l), pg.Value(l))), Point2D(r, max(pf.Value(r), pg.Value(r)))));
            pf.domain.left = r;
            pg.domain.left = r;
            if (epsilon_equal(r, pf.domain.right)) ++i;
            if (epsilon_equal(r, pg.domain.right)) ++j;
        }
    }
    for (; i < f.PieceCount(); ++i) h.AddPiece(f.Piece(i));
    for (; j < g.PieceCount(); ++j) h.AddPiece(g.Piece(j));
    return h;
}

PWLFunction Max(const PWLFunction& f, double a)
{
    return Max(f, PWLFunction::ConstantFunction(a, f.Domain()));
}

PWLFunction Max(double a, const PWLFunction& f)
{
    return Max(f, a);
}

PWLFunction Min(const PWLFunction& f, const PWLFunction& g)
{
    return Max(f*-1.0, g*-1.0)*-1.0;
}

PWLFunction Min(const PWLFunction& f, double a)
{
    return Min(f, PWLFunction::ConstantFunction(a, f.Domain()));
}

PWLFunction Min(double a, const PWLFunction& f)
{
    return Min(f, a);
}

std::size_t PWLFunction::memory_footprint_bytes() const {
    // sizeof(*this) covers pieces_ header, any other members
    return sizeof(*this)
         + pieces_.capacity() * sizeof(LinearFunction);
}

double PWLFunction::compute_area() const 
{
    double area = 0.0;
    for (const auto& piece : pieces_) {
        // Area under the piece is the trapezoidal area
        // - base1 is y1 (the y-value at the start of the segment)
        // - base2 is y2 (the y-value at the end of the segment)
        // - height is (x2 - x1) (the width of the segment along the x-axis)
        area += ((piece.Value(piece.domain.left) + piece.Value(piece.domain.right)) * (piece.domain.right - piece.domain.left)) / 2.0;
    }
    return area;
}


} // namespace goc