/*
 * Copyright (C) 2014 IRSTV CNRS-FR-2488
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.h2gis.h2spatialext.function.spatial.topography;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import org.h2gis.h2spatialapi.DeterministicScalarFunction;
import org.jdelaunay.delaunay.error.DelaunayError;
import org.jdelaunay.delaunay.geometries.DEdge;
import org.jdelaunay.delaunay.geometries.DPoint;
import org.jdelaunay.delaunay.geometries.DTriangle;

/**
 * Extract the ridge line of a triangle
 * @author Erwan Bocher
 */
public class ST_RidgeLine  extends DeterministicScalarFunction{

    
    public ST_RidgeLine(){
        addProperty(PROP_REMARKS, "Extract the ridge line of a triangle.");
    }
    
    @Override
    public String getJavaStaticMethod() {
        return "computeRidgeLine";
    }
    
    /**
     * This method is used to compute the ridge line on a triangle
     *
     * @param geometry
     * @return
     * @throws DelaunayError if the geometry is not a triangle or malformed.
     */
    public static LineString computeRidgeLine(Geometry geometry) throws DelaunayError {

        DTriangle dTriangle = TINFeatureFactory.createDTriangle(geometry);

        GeometryFactory factory = geometry.getFactory();

        DEdge[] edges = dTriangle.getEdges();

        DEdge edge1 = edges[0];
        DEdge edge2 = edges[1];
        DEdge edge3 = edges[2];

        boolean goToEdge1 = dTriangle.isTopoOrientedToEdge(edge1);
        boolean goToEdge2 = dTriangle.isTopoOrientedToEdge(edge2);
        boolean goToEdge3 = dTriangle.isTopoOrientedToEdge(edge3);
        LineString ridgeLine = null;
        DPoint pt;
        DPoint ptRidge;
        if (goToEdge1) {
            if (goToEdge2) {
                // the ridge crosses edge1 and edge2
                pt = dTriangle.getOppositePoint(edge3);
                ptRidge = dTriangle.getCounterSteepestIntersection(pt);
                ridgeLine = createLine(ptRidge, pt, factory);
            } else if (goToEdge3) {
                // the ridge crosses edge1 and edge3
                pt = dTriangle.getOppositePoint(edge2);
                ptRidge = dTriangle.getCounterSteepestIntersection(pt);
                ridgeLine = createLine(ptRidge, pt, factory);
            } else {
                pt = dTriangle.getOppositePoint(edge1);
                ridgeLine = createLine(edge1.getMiddle(), pt, factory);
            }
        } else if (goToEdge2 && goToEdge3) {
            // the ridge crosses edge2 and edge3
            pt = dTriangle.getOppositePoint(edge1);
            ptRidge = dTriangle.getCounterSteepestIntersection(pt);
            ridgeLine = createLine(ptRidge, pt, factory);
        }
        return ridgeLine;
    }
    
    /**
     * Create a linestring
     * @param startPoint
     * @param endPoint
     * @param factory
     * @return 
     */
    public static LineString createLine(DPoint startPoint, DPoint endPoint, GeometryFactory factory) {
        return factory.createLineString(new Coordinate[]{TINFeatureFactory.dPointToCoordinate(startPoint),
            TINFeatureFactory.dPointToCoordinate(endPoint)});
    }
    
}
