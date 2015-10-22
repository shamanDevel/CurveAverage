/*
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package curveavg;

import org.apache.commons.math3.analysis.solvers.NewtonRaphsonSolver;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.junit.Assert;
import org.junit.Test;

public final class SolverTest {

    @Test
    public void testQuinticZero_NewtonRaphsonFailureCase() {
    
        // Solve using NewtonRaphson and count the number of results
        float c[] = {-38.764412f, 76.10117f, -56.993206f, 28.603401f, -10.2824955f, 1.0f};
        MedialAxisTransform.SolverResult res = MedialAxisTransform.solveQuinticNewtonRaphson(c[0], c[1], c[2], c[3], c[4], c[5]);
        int rootsNR = 0;
        for(int i = 0; i < res.im.length; i++) {
            if(Math.abs(res.im[i]) < 1e-3) rootsNR++;
        }
        System.out.println("# NR roots: " + rootsNR);

        // Solve using NewtonRaphson and count the number of results
        MedialAxisTransform.SolverResult res2 = MedialAxisTransform.solveQuinticLaguerre(c[0], c[1], c[2], c[3], c[4], c[5]);
        int rootsL = 0;
        for(int i = 0; i < res.im.length; i++) {
            if(Math.abs(res2.im[i]) < 1e-3) rootsL++;
        }
        System.out.println("# L roots: " + rootsL);

        // Solve using Laguerre
        double dt = 0.01;
        double coefficients[] = new double [6];
        for(int i = 0; i < 6; i++) coefficients[i] = c[5-i];
        PolynomialFunction f = new PolynomialFunction(coefficients);
        LaguerreSolver solver = new LaguerreSolver();
        for(double t = 0; t <= (1.0-dt); t += dt) {
 
            // Check if there is a sign-crossing between the two times
            double t2 = t + dt;
            double f1 = f.value(t);
            double f2 = f.value(t2);
            if(Math.signum(f1) == Math.signum(f2)) continue;
            
            // Compute using Laguerre
            double result = solver.solve(100, f, t, t2);
            System.out.println("Result: " + result);
        }
        
        
        
    }

    /*
    @Test
    public void testQuinticZero() {
        
        UnivariateDifferentiableFunction f = new QuinticFunction(1, 0.0, -5/4.0, 0.0, 0.25, 0.0);
        double result;

        NewtonRaphsonSolver solver = new NewtonRaphsonSolver();
        result = solver.solve(100, f, -0.19, -0.2);

        Assert.assertEquals(result, 0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, -0.1, 0.3);
        Assert.assertEquals(result, 0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, -0.3, 0.45);
        Assert.assertEquals(result, 0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.3, 0.7);
        Assert.assertEquals(result, 0.5, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.2, 0.6);
        Assert.assertEquals(result, 0.5, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.05, 0.95);
        Assert.assertEquals(result, 0.5, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.85, 1.25);
        Assert.assertEquals(result, 1.0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.8, 1.2);
        Assert.assertEquals(result, 1.0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.85, 1.75);
        Assert.assertEquals(result, 1.0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.55, 1.45);
        Assert.assertEquals(result, 1.0, solver.getAbsoluteAccuracy());

        result = solver.solve(100, f, 0.85, 5);
        Assert.assertEquals(result, 1.0, solver.getAbsoluteAccuracy());

    }
    */
}
