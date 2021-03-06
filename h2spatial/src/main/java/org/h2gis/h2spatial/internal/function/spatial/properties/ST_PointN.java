/**
 * h2spatial is a library that brings spatial support to the H2 Java database.
 *
 * h2spatial is distributed under GPL 3 license. It is produced by the "Atelier SIG"
 * team of the IRSTV Institute <http://www.irstv.fr/> CNRS FR 2488.
 *
 * Copyright (C) 2007-2012 IRSTV (FR CNRS 2488)
 *
 * h2patial is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * h2spatial is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * h2spatial. If not, see <http://www.gnu.org/licenses/>.
 *
 * For more information, please consult: <http://www.orbisgis.org/>
 * or contact directly:
 * info_at_ orbisgis.org
 */

package org.h2gis.h2spatial.internal.function.spatial.properties;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;

import org.h2gis.h2spatialapi.DeterministicScalarFunction;

import java.sql.SQLException;

/**
 * Returns the N point of a LINESTRING geometry as a POINT or NULL if the
 * input parameter is not a LINESTRING.
 * As the OGC specs ST_PointN is 1-N based.
 * @author Nicolas Fortin
 */
public class ST_PointN extends DeterministicScalarFunction {
    private static final String OUT_OF_BOUNDS_ERR_MESSAGE = "ST_PointN index > ST_NumPoints or index <= 0, Point index must be in the range [1-NbPoints]";

    /**
     * Default constructor
     */
    public ST_PointN() {
        addProperty(PROP_REMARKS, "Returns the N point of a LINESTRING geometry as a POINT or NULL if the input parameter is not a LINESTRING.As the OGC specs ST_PointN is 1-N based.");
    }

    @Override
    public String getJavaStaticMethod() {
        return "getPointN";
    }

    /**
     * @param geometry Geometry instance
     * @param pointIndex Point index [1-NbPoints]
     * @return Point geometry or null if geometry is null
     * @throws SQLException if index is out of bound.
     */
    public static Geometry getPointN(Geometry geometry,int pointIndex) throws SQLException {
        if (geometry instanceof MultiLineString) {
            if (geometry.getNumGeometries() == 1) {
                LineString line = (LineString) geometry.getGeometryN(0);
                if(pointIndex<=0 || pointIndex <= line.getNumPoints()) {
                    return line.getPointN(pointIndex-1);
                } else {
                    throw new SQLException(OUT_OF_BOUNDS_ERR_MESSAGE);
                }
            }
        } else if (geometry instanceof LineString) {
            LineString line = (LineString) geometry;
            if(pointIndex<=0 || pointIndex <= line.getNumPoints()) {
                return line.getPointN(pointIndex-1);
            }else {
                throw new SQLException(OUT_OF_BOUNDS_ERR_MESSAGE);
            }
        }
        return null;
    }
}
