/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package curveavg;

import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;

/**
 *
 * @author cerdogan
 */

class QuinticFunction implements UnivariateDifferentiableFunction {

    public double a, b, c, d, e, f;
    QuinticFunction (double a_, double b_, double c_, double d_, double e_, double f_) {
        a = a_;
        b = b_;
        c = c_;
        d = d_;
        e = e_;
        f = f_;
    }
    
    public double value(double x) {
        return (a * Math.pow(x,5) + b * Math.pow(x,4) + c * Math.pow(x,3) + d * Math.pow(x,2) + e * x + f);
    }   

    public DerivativeStructure value(DerivativeStructure t) {
        return t.pow(5).multiply(a).add(
                t.pow(4).multiply(b).add(
                        t.pow(3).multiply(c).add(
                                t.pow(2).multiply(d).add(
                                        t.multiply(e).add(f)))));
    }   

}